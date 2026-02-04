import re
import json
import logging
from dataclasses import dataclass
from typing import List, Dict, Any, Optional, Callable

from agent.llm_client import LLMClient
from agent.prompts import SYSTEM_SKILL_WRITER, SYSTEM_SELF_VERIFIER

logger = logging.getLogger(__name__)

# Regex to extract Python code blocks
_CODE_BLOCK_RE = re.compile(r"```python\s*(.*?)\s*```", re.DOTALL | re.IGNORECASE)
_ANY_CODE_BLOCK_RE = re.compile(r"```\s*(.*?)\s*```", re.DOTALL)
_SKILL_CALL_RE = re.compile(r"\bskills\s*\.")


@dataclass
class PlanResult:
    code: str
    raw: str
    metadata: Optional[Dict[str, Any]] = None


@dataclass
class ReflectionResult:
    analysis: str
    root_cause: str
    suggested_fixes: List[str]
    should_retry: bool


def _extract_python_code(text: str) -> Optional[str]:
    """Extract Python code from markdown code blocks."""
    match = _CODE_BLOCK_RE.search(text)
    if match:
        return match.group(1).strip()
    
    match = _ANY_CODE_BLOCK_RE.search(text)
    if match:
        return match.group(1).strip()
    
    return None




class Planner:
    """
    LLM-based planner for generating robot skill code.
    
    - Init skill generation from task description
    - Self-reflection on failures
    - Skill refinement based on feedback
    """

    def __init__(self, llm: LLMClient):
        self.llm = llm
        self._code_callback: Optional[Callable] = None
    
    def set_code_callback(self, callback: Callable):
        self._code_callback = callback
    
    def propose_skill_code(
        self,
        task_name: str,
        task_description: str,
        observation: Dict[str, Any],
        available_skills: List[Dict[str, Any]],
        attempt: int,
        last_error: Optional[str] = None,
        reflection: Optional[ReflectionResult] = None,
    ) -> PlanResult:
        """
        Generate skill code for a task.
        
        Args:
            task_name: Name of the task
            task_description: What the skill should accomplish
            observation: Current robot/environment state
            available_skills: Previously learned skills for reference
            attempt: Current attempt number
            last_error: Error from previous attempt 
            reflection: Self-reflection on failure
        """
        # Format available skills
        skills_brief = self._format_skills(available_skills)
        
        # Build user prompt
        user_prompt = f"""Task:
- name: {task_name}
- description: {task_description}

Current Observation (robot state):
{json.dumps(observation, indent=2, default=str)}

Available skills you may reference (for patterns/ideas):
{skills_brief}

Attempt: {attempt}
"""
        
        # Add error context if retrzying
        if last_error:
            user_prompt += f"""
Previous attempt failed with error:
{last_error}
"""
        
        # Add reflection
        if reflection:
            user_prompt += f"""
Analysis of previous failure:
- Root cause: {reflection.root_cause}
- Suggested fixes: {', '.join(reflection.suggested_fixes)}
"""
        
        user_prompt += """
Write ONE NEW SKILL for this task.
Requirements:
- Return True on success, False on failure.
- Use robot.move_ee(...) with pos_tolerance >= 0.06 for approach/descend.
- For LIFT: use pos_tolerance=0.03 and lift to z=0.20 (higher than needed!)
- Gripper is physics-based - close_gripper() stops when touching object!
- Check cube position after lift to verify success, not move_ee return value.
- Include robot.log() calls for debugging.
- Make the skill reusable: use kwargs like cube_name / object_name and target_xy / target_xyz.
- If the task names a specific cube/target, set those as DEFAULTS in kwargs.
- Skill name should be generic (no cube indices like cube1/cube2 in the name).
- Output MUST be exactly one python code block.
"""
        
        # Generate code
        raw = self.llm.chat(
            messages=[
                {"role": "system", "content": SYSTEM_SKILL_WRITER},
                {"role": "user", "content": user_prompt},
            ],
            temperature=0.15 + (attempt * 0.1),  # Increase temperature on retry
            max_tokens=2500,  # Increased for complex skill
            call_type="skill_generation",
        )
        
        code = _extract_python_code(raw) or ""
        
        # Notify callback of extracted code
        if self._code_callback:
            self._code_callback(code)

        if not code:
            logger.warning("LLM did not return a valid code block")
            return PlanResult(code="", raw=raw)

        

        return PlanResult(code=code, raw=raw)

    def reflect_on_failure(
        self,
        code: str,
        error: str,
        pre_state: Dict[str, Any],
        post_state: Dict[str, Any],
    ) -> Optional[ReflectionResult]:
        """
        Analyze why a skill failed and suggest fixes
        
        Args:
            code: The failed skill code
            error: Error message or failure reason
            pre_state: Robot state before execution
            post_state: Robot state after execution
        """
        user_prompt = f"""Analyze this failed robot skill:

FAILED CODE:
```python
{code}
```

ERROR/FAILURE:
{error}

STATE BEFORE EXECUTION:
{json.dumps(pre_state, indent=2, default=str)}

STATE AFTER EXECUTION:
{json.dumps(post_state, indent=2, default=str)}

Analyze what went wrong and how to fix it.
Respond with JSON containing: analysis, root_cause, suggested_fixes (list), should_retry (bool)
"""
        
        try:
            response = self.llm.chat_json(
                messages=[
                    {"role": "system", "content": SYSTEM_SELF_VERIFIER},
                    {"role": "user", "content": user_prompt},
                ],
                temperature=0.1,
                max_tokens=800,
                call_type="reflection",
            )
            
            return ReflectionResult(
                analysis=response.get("analysis", ""),
                root_cause=response.get("root_cause", "Unknown"),
                suggested_fixes=response.get("suggested_fixes", []),
                should_retry=response.get("should_retry", True),
            )
            
        except Exception as e:
            logger.warning(f"Reflection failed: {e}")
            return None

    def _format_skills(self, skills: List[Dict[str, Any]], max_skills: int = 12) -> str:
        """Format skill list for prompt."""
        if not skills:
            return "(none available)"
        
        lines = []
        for skill in skills[:max_skills]:
            name = skill.get("name", "unnamed")
            desc = skill.get("description", "")[:100]
            tags = skill.get("tags", [])
            lines.append(f"- {name}: {desc}; tags={tags}")
        
        return "\n".join(lines)
