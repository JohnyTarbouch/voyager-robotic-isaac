import logging
import re
from typing import Any, Dict, Optional, Callable, List

from agent.skill_library import SkillLibrary

logger = logging.getLogger(__name__)


def _to_snake_case(name: str) -> str:
    s = re.sub(r'(?<=[a-z0-9])(?=[A-Z])', '_', name)
    return s.lower().replace(' ', '_').replace('-', '_')


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
        self._skills_proxy = None
    
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
                skill_fn = self._create_skill_function(skill_name, skill.code, skill.accepted_kwargs)
                if skill_fn:
                    skills[skill_name] = skill_fn
        
        return skills
    
    def _create_skill_function(
        self,
        name: str,
        code: str,
        accepted_kwargs: Optional[List[str]] = None,
    ) -> Optional[Callable]:
        """Create a callable func from skill code"""
        try:
            # Compile the skill code
            compiled = compile(code, f"<skill:{name}>", "exec")
            
            # Create namespace with safe builtins
            namespace = self._get_safe_namespace()
            if self._skills_proxy is not None:
                namespace["skills"] = self._skills_proxy
            
            # Execute to define the run func
            exec(compiled, namespace)
            
            if "run" not in namespace or not callable(namespace["run"]):
                logger.warning(f"Skill '{name}' has no run() function")
                return None
            
            run_fn = namespace["run"]
            accepted_kwargs_set = set(accepted_kwargs or [])
            
            # Create wrapper to binds robot
            def skill_wrapper(**kwargs) -> bool:
                if accepted_kwargs_set:
                    unknown = sorted(set(kwargs.keys()) - accepted_kwargs_set)
                    if unknown:
                        logger.error(
                            "Skill '%s' received unsupported kwargs %s. Accepted kwargs: %s",
                            name,
                            unknown,
                            sorted(accepted_kwargs_set),
                        )
                        return False
                return run_fn(self.robot, **kwargs)
            
            # Add metadata
            skill_wrapper.__name__ = name
            skill_wrapper.__doc__ = namespace.get("SKILL_METADATA", {}).get("description", "")
            
            return skill_wrapper
            
        except Exception as e:
            logger.warning(f"Failed to create function for skill '{name}': {e}")
            return None

    def set_skills_proxy(self, skills_proxy: Any) -> None:
        """Inject shared skills proxy for nested skill composition."""
        self._skills_proxy = skills_proxy
    
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
        norm = _to_snake_case(skill_name)

        if norm in self._skill_cache:
            return self._skill_cache[norm](**kwargs)
        
        #normalized name
        skill = self.skill_library.get(norm) or self.skill_library.get(skill_name)
        if not skill:
            logger.error(f"Skill not found: {skill_name} (normalized: {norm})")
            return False
        
        skill_fn = self._create_skill_function(norm, skill.code, skill.accepted_kwargs)
        if not skill_fn:
            return False
        
        self._skill_cache[norm] = skill_fn
        
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
    
    # Create base context
    context = executor._get_safe_namespace()
    
    class SkillsProxy:
        def __init__(self, skill_dict: Dict[str, Callable], executor: SkillExecutor):
            self._skills = skill_dict
            self._executor = executor
        
        def __getattr__(self, name: str) -> Callable:
            if name.startswith("_"):
                raise AttributeError(name)
            norm = _to_snake_case(name)
            if norm in self._skills:
                logger.info("[SkillsProxy] getattr('%s') -> found as '%s'", name, norm)
                return self._skills[norm]
            if name in self._skills:
                logger.info("[SkillsProxy] getattr('%s') -> found (exact match)", name)
                return self._skills[name]
            logger.warning("[SkillsProxy] getattr('%s') -> NOT FOUND", name)
            raise AttributeError(f"Skill not found: {name}")
        
        def call(self, skill_name: str, **kwargs) -> bool:
            logger.info("[SkillsProxy] call('%s', %s) -> executing", skill_name, list(kwargs.keys()))
            result = self._executor.call_skill(skill_name, **kwargs)
            logger.info("[SkillsProxy] call('%s') -> returned %s", skill_name, result)
            return result
        
        def list(self) -> list:
            return list(self._skills.keys())
        
        def has(self, name: str) -> bool:
            norm = _to_snake_case(name)
            found = norm in self._skills or name in self._skills
            logger.info("[SkillsProxy] has('%s') -> %s (normalized: '%s')", name, found, norm)
            return found
    
    # Build proxy first so nested skills.call works inside composed skills.
    skills: Dict[str, Callable] = {}
    skills_proxy = SkillsProxy(skills, executor)
    executor.set_skills_proxy(skills_proxy)
    skills.update(executor.get_available_skills())

    context["skills"] = skills_proxy
    
    return context
