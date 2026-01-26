import ast
import logging
from typing import Any, Dict, Optional, Callable

from agent.skill_library import SkillLibrary

logger = logging.getLogger(__name__)


class SkillExecutor:
    """
    Executor that provides learned skills as callable functions.
    
    This enables skill composition meaninig new skills can call previously
    learned skills instead of reimplementing everything from scratch
    """
    
    def __init__(self, skill_library: SkillLibrary, robot: Any):
        """
        Initialize the skill executor.
        
        Args:
            skill_library: Library of learned skills
            robot: Robot API instance
        """
        self.skill_library = skill_library
        self.robot = robot
        self._skill_cache: Dict[str, Callable] = {}
    
    def get_available_skills(self) -> Dict[str, Callable]:
        """
        Get all available skills as callable functions.
        
        Returns:
            Dict mapping skill names to callable functions
        """
        skills = {}
        
        for skill_info in self.skill_library.list():
            skill_name = skill_info["name"]
            skill = self.skill_library.get(skill_name)
            
            if skill and skill.code:
                skill_fn = self._create_skill_function(skill_name, skill.code)
                if skill_fn:
                    skills[skill_name] = skill_fn
        
        return skills
    
    def _create_skill_function(self, name: str, code: str) -> Optional[Callable]:
        """Create a callable func from skill code"""
        try:
            # Compile the skill code
            compiled = compile(code, f"<skill:{name}>", "exec")
            
            # Create namespace with safe builtins
            namespace = self._get_safe_namespace()
            
            # Execute to define the run func
            exec(compiled, namespace)
            
            if "run" not in namespace or not callable(namespace["run"]):
                logger.warning(f"Skill '{name}' has no run() function")
                return None
            
            run_fn = namespace["run"]
            
            # Create wrapper to binds robot
            def skill_wrapper(**kwargs) -> bool:
                return run_fn(self.robot, **kwargs)
            
            # Add metadata
            skill_wrapper.__name__ = name
            skill_wrapper.__doc__ = namespace.get("SKILL_METADATA", {}).get("description", "")
            
            return skill_wrapper
            
        except Exception as e:
            logger.warning(f"Failed to create function for skill '{name}': {e}")
            return None
    
    def _get_safe_namespace(self) -> Dict[str, Any]:
        """Get a safe namespace for skill execution."""
        import builtins
        
        safe_builtins = {
            "abs": abs, "all": all, "any": any, "bool": bool,
            "dict": dict, "enumerate": enumerate, "float": float,
            "int": int, "len": len, "list": list, "max": max,
            "min": min, "range": range, "str": str, "sum": sum,
            "tuple": tuple, "zip": zip, "print": print,
            "round": round, "sorted": sorted, "reversed": reversed,
            "isinstance": isinstance, "type": type,
            "True": True, "False": False, "None": None,
        }
        
        # Add math functions
        import math
        safe_builtins["sqrt"] = math.sqrt
        safe_builtins["sin"] = math.sin
        safe_builtins["cos"] = math.cos
        safe_builtins["pi"] = math.pi
        
        return {"__builtins__": safe_builtins}
    
    def call_skill(self, skill_name: str, **kwargs) -> bool:
        """
        Call a learned skill by name.
        
        Args:
            skill_name: Name of the skill to call
            kwargs: Arguments to pass to the skill  (position for example)
            
        Returns:
            True if skill succeeded, False otherwise
        """
        # Check cache first
        if skill_name in self._skill_cache:
            return self._skill_cache[skill_name](**kwargs)
        
        # Load skill
        skill = self.skill_library.get(skill_name)
        if not skill:
            logger.error(f"Skill not found: {skill_name}")
            return False
        
        skill_fn = self._create_skill_function(skill_name, skill.code)
        if not skill_fn:
            return False
        
        self._skill_cache[skill_name] = skill_fn
        
        return skill_fn(**kwargs)
    
    def clear_cache(self):
        self._skill_cache.clear()


def create_skill_context(skill_library: SkillLibrary, robot: Any) -> Dict[str, Any]:
    """
    Create a context dict with all learned skills available as functions
    
    This context can be passed to exec() when running new skill code
    allowing the new code to call previously learned skills
    
    Args:
        skill_library: Library of learned skills
        robot: Robot API instance
        
    Returns:
        Dict with skill functions and safe builtins
    """
    executor = SkillExecutor(skill_library, robot)
    skills = executor.get_available_skills()
    
    # Create base context
    context = executor._get_safe_namespace()
    
    class SkillsProxy:
        """Proxy object for accessing learned skills"""
        def __init__(self, skill_dict: Dict[str, Callable], executor: SkillExecutor):
            self._skills = skill_dict
            self._executor = executor
        
        def __getattr__(self, name: str) -> Callable:
            if name.startswith("_"):
                raise AttributeError(name)
            if name in self._skills:
                return self._skills[name]
            raise AttributeError(f"Skill not found: {name}")
        
        def call(self, skill_name: str, **kwargs) -> bool:
            return self._executor.call_skill(skill_name, **kwargs)
        
        def list(self) -> list:
            return list(self._skills.keys())
        
        def has(self, name: str) -> bool:
            return name in self._skills
    
    context["skills"] = SkillsProxy(skills, executor)
    
    return context