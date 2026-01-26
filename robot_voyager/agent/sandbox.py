"""Safe execution sandbox for generated skill code"""

import ast
import builtins
import traceback
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Dict, Set, Optional


@dataclass
class ExecutionResult:
    """Result of executing skill code in sandbox."""
    ok: bool
    returned: Any = None
    error: str | None = None


# Names banned
BANNED_NAMES: Set[str] = {
    "open", "exec", "eval", "compile", "__import__", "input",
    "os", "sys", "subprocess", "socket", "pathlib", "shutil",
    "globals", "locals", "vars", "dir", "getattr", "setattr", "delattr",
    "breakpoint", "exit", "quit",
}

# Allowed functions
ALLOWED_BUILTINS: Set[str] = {
    "abs", "all", "any", "bool", "dict", "enumerate", "float", "int",
    "len", "list", "max", "min", "range", "str", "sum", "tuple", "zip",
    "print", "round", "sorted", "reversed", "isinstance", "type",
    "True", "False", "None",
}


def _validate_skill_structure(code: str) -> None:
    """Validate that skill code has correct structure."""
    try:
        tree = ast.parse(code)
    except SyntaxError as e:
        raise ValueError(f"Syntax error: {e}")
    
    module_functions = []
    run_function = None
    
    for node in ast.iter_child_nodes(tree):
        if isinstance(node, ast.FunctionDef):
            module_functions.append(node.name)
            if node.name == "run":
                run_function = node
    
    if not run_function:
        raise ValueError("Skill code must define: def run(robot, **kwargs) -> bool")
    
    # Check if run() calls any other modulelevel functions
    # These would fail at runtime since we only exec run()
    calls_in_run = set()
    for node in ast.walk(run_function):
        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name):
                calls_in_run.add(node.func.id)
    
    # Find helper functions that are called but defined outside run()
    helper_functions = set(module_functions) - {"run"}
    invalid_calls = calls_in_run & helper_functions
    
    if invalid_calls:
        raise ValueError(
            f"Helper functions defined outside run() won't work: {invalid_calls}. "
            f"Move helper logic inside run() or use nested functions."
        )

def _validate_ast(code: str) -> None:
    """Validate code AST for safety violations"""
    try:
        tree = ast.parse(code)
    except SyntaxError as e:
        raise ValueError(f"Syntax error in code: {e}")
    
    for node in ast.walk(tree):
        # Block all imports
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            raise ValueError("Imports are not allowed in skills.")
        
        # Block banned function calls
        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name) and node.func.id in BANNED_NAMES:
                raise ValueError(f"Banned function call: {node.func.id}")
            # Block __methods__
            if isinstance(node.func, ast.Attribute) and node.func.attr.startswith("__"):
                raise ValueError(f"Attribute access not allowed: {node.func.attr}")
        
        # Block banned names
        if isinstance(node, ast.Name) and node.id in BANNED_NAMES:
            raise ValueError(f"Banned name: {node.id}")
        
        # Block attribute access
        if isinstance(node, ast.Attribute):
            if node.attr.startswith("__") and node.attr.endswith("__"):
                raise ValueError(f"Attribute access not allowed: {node.attr}")
    _validate_skill_structure(code)

def _create_safe_globals(extra_context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    safe_builtins = {}
    for name in ALLOWED_BUILTINS:
        if hasattr(builtins, name):
            safe_builtins[name] = getattr(builtins, name)
    
    # Add math function
    import math
    safe_builtins["sqrt"] = math.sqrt
    safe_builtins["sin"] = math.sin
    safe_builtins["cos"] = math.cos
    safe_builtins["pi"] = math.pi
    
    globals_dict = {"__builtins__": MappingProxyType(safe_builtins)}
    
    if extra_context:
        for key, value in extra_context.items():
            if key != "__builtins__":
                globals_dict[key] = value
    
    return globals_dict


def _execute_code(
    code: str, 
    robot: Any, 
    kwargs: Dict[str, Any],
    skill_context: Optional[Dict[str, Any]] = None
) -> Any:
    """Execute validated code in environment"""
    _validate_ast(code)
    
    globals_dict = _create_safe_globals(skill_context)
    locals_dict: Dict[str, Any] = {}
    
    # Compile and execute
    compiled = compile(code, "<skill>", "exec")
    exec(compiled, globals_dict, locals_dict)
    
    # Verify run function exists
    if "run" not in locals_dict or not callable(locals_dict["run"]):
        raise ValueError("Skill code must define: def run(robot, **kwargs) -> bool")
    
    # Execute the run function
    return locals_dict["run"](robot, **kwargs)


def run_skill(
    code: str, 
    robot: Any, 
    kwargs: Dict[str, Any] = None,
    timeout_s: int = 30,
    skill_context: Optional[Dict[str, Any]] = None
) -> ExecutionResult:
    """
    Execute skill code in a sandboxed environment.
    
    Args:
        code: Python code defining SKILL_METADATA and run(robot, **kwargs)
        robot: RobotAPI 
        kwargs: Additional arguments to pass to run(), pos/obs for example
        timeout_s: Timeout for execution
        skill_context: Learned skills available
        
    Returns:
        ExecutionResult with success status and any errors
    """
    if kwargs is None:
        kwargs = {}
    
    if hasattr(robot, "set_deadline_s"):
        robot.set_deadline_s(timeout_s)
    
    try:
        result = _execute_code(code, robot, kwargs, skill_context)
        return ExecutionResult(ok=True, returned=result)
        
    except ValueError as e:
        return ExecutionResult(ok=False, error=f"Validation error: {e}")
        
    except Exception:
        return ExecutionResult(ok=False, error=traceback.format_exc())
        
    finally:
        if hasattr(robot, "clear_deadline"):
            robot.clear_deadline()


def extract_skill_metadata(code: str) -> Dict[str, Any]:
    try:
        _validate_ast(code)
        
        globals_dict = _create_safe_globals()
        locals_dict: Dict[str, Any] = {}
        
        compiled = compile(code, "<skill>", "exec")
        exec(compiled, globals_dict, locals_dict)
        
        meta = locals_dict.get("SKILL_METADATA", {})
        if isinstance(meta, dict):
            return {
                "name": str(meta.get("name", "unnamed")),
                "description": str(meta.get("description", "")),
                "tags": list(meta.get("tags", [])),
            }
        return {}
        
    except Exception:
        return {}