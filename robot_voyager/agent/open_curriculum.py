"""
Open Ended Curriculum autonomous task

The agent proposes its own tasks based on:
1. What skills it has learned
2. What objects are available
3. What it wants to explore

No predefined task list -> creativity
"""

import logging
import json
from typing import List, Optional, Dict, Any
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class OpenTask:
    """
    A task proposed by the LLM -> can be anything
        """
    name: str
    description: str
    success_criteria: str
    difficulty: int = 5
    
    def __str__(self):
        return f"{self.name}: {self.description}"


OPEN_CURRICULUM_PROMPT = """You are an autonomous robot learning agent. Your goal is to continuously learn new skills by proposing interesting tasks.

You control a Franka robot arm with a gripper. You have access to these objects:
{objects}

Your current skill library, do not write the same skill multiple times, write new skills as you learn them:
{skills}

Tasks you have already completed:
{completed_tasks}

Tasks that failed (avoid repeating these exact tasks):
{failed_tasks}

GUIDELINES FOR PROPOSING TASKS:
0. **NEVER propose a task that is already in the 'completed tasks' list above!** Pick something NEW.
1. Start simple, gradually increase complexity
2. Build on skills you've already learned
3. Be CREATIVE! The environment has 4 colored cubes and an open-top box - explore possibilities!
CRITICAL - SUCCESS CRITERIA TOLERANCES:
- Robot has ~0.05m position offset - NEVER use tolerances < 0.05m!
- For single-cube position checks, use 0.06-0.08m tolerance
- For multi-object structure checks (pyramid base distance, tower alignment), use **0.10m tolerance**
  because EACH cube drifts ~0.05m, so two cubes can drift 0.10m relative to each other!
- Example good criteria: "cube within 0.08m of target position"
- Example BAD criteria: "cube within 0.01m of target" (impossible!)
4. **MANDATORY TASK PROGRESSION** - propose the FIRST uncompleted step below!
   Check the 'completed tasks' list and find which step has NOT been done yet. Propose THAT step.
   Do NOT skip ahead! Do not write the same task with a different name.
   a) pick-and-place one cube to a new position
   b) pick a cube and place it inside the open-top box (use robot.get_box_position())
   c) stack 2 cubes into a tower
   d1) line patterns
   d2) full 3-cube pyramid (2 cubes as base + 1 cube on top and elevated)
       IMPORTANT: this is NOT a "pyramid base" task.
       If d2 is pending, do not propose base-only pyramid tasks.
   d3) stack 3-4 cubes, L-shapes, sorting
   e) move all cubes to one side
5. Challenge yourself but stay within robot capabilities:
   - Robot can reach: x: 0.3-0.7, y: -0.4-0.4, z: 0.0-0.6
   - Gripper can grasp objects ~0.05m wide
   - Cubes are 0.05m x 0.05m x 0.05m
6. Use generic task names (no cube indices like cube1/cube2 in the name).
   - If a specific cube is needed, mention it in the description and success criteria.
   - Avoid proposing multiple tasks that only differ by cube index.
7. NEW FREE SPACE: For any placement or stacking task, choose target positions in a clearly free area,
   far from current cube positions (>= 0.10m if possible), and avoid reusing current cube XY locations.
8. For box tasks, do NOT hardcode box coordinates - use robot.get_box_position() or say "cube is inside the box" in success criteria.


Respond with JSON only:
{{
    "task_name": "descriptive_name",
    "description": "Detailed description of what to do",
    "success_criteria": "How to verify the task succeeded",
    "difficulty": 1-10,
    "reasoning": "Why this is a good next task to learn"
}}
"""


class OpenEndedCurriculum:
    def __init__(self, llm_client, max_tasks: int = 50):
        """
        Initialize open-ended curriculum.
        
        Args:
            llm_client: LLM client for task proposal
            max_tasks: prevents infinite loops
        """
        self.llm = llm_client
        self.max_tasks = max_tasks
        
        self._completed_tasks: List[str] = []
        self._failed_tasks: List[str] = []
        self._task_count = 0
        self._skills: List[str] = []
        self._dedup_retries = 0
    
    def set_skills(self, skills: List[str]):
        """Update the list of learned skills"""
        self._skills = skills
    
    def add_completed(self, task_name: str):
        """Record completed task"""
        self._completed_tasks.append(task_name)
    
    def add_failed(self, task_name: str):
        """Record failed task"""
        self._failed_tasks.append(task_name)
    
    def propose_next_task(self, observation: Dict[str, Any], _is_retry: bool = False) -> Optional[OpenTask]:
        """
        Use LLM to propose the next task based on current state
        """
        if not _is_retry:
            self._dedup_retries = 0
        self._task_count += 1
        
        # limit
        if self._task_count > self.max_tasks:
            logger.info(f"Reached max tasks limit ({self.max_tasks})")
            return None
        
        objects_str = "None detected"
        if "objects" in observation:
            objects_list = []
            for name, info in observation["objects"].items():
                if isinstance(info, dict):
                    pos = info.get("position", (0, 0, 0))
                else:
                    pos = info
                objects_list.append(f"  - {name} at position ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            objects_str = "\n".join(objects_list)
        
        
        # Format skills
        skills_str = "None learned yet" if not self._skills else "\n".join(f"  - {s}" for s in self._skills[-10:])
        
        # Format completed/failed
        completed_str = "None" if not self._completed_tasks else "\n".join(f"  - {t}" for t in self._completed_tasks[-10:])
        failed_str = "None" if not self._failed_tasks else "\n".join(f"  - {t}" for t in self._failed_tasks[-5:])
        
        prompt = OPEN_CURRICULUM_PROMPT.format(
            objects=objects_str,
            skills=skills_str,
            completed_tasks=completed_str,
            failed_tasks=failed_str
        )
        
        try:
            response = self.llm.chat(
                messages=[
                    {"role": "system", "content": "You are an autonomous robot learning agent. Propose creative, achievable tasks."},
                    {"role": "user", "content": prompt},
                ],
                temperature=0.7,# Higher temperature for creativity
                max_tokens=800,
                call_type="curriculum_proposal",
            )
            
            # Parse JSON
            import re
            json_match = re.search(r'\{[^{}]*\}', response, re.DOTALL)
            if json_match:
                data = json.loads(json_match.group())
                
                task = OpenTask(
                    name=data.get("task_name", f"task_{self._task_count}"),
                    description=data.get("description", "No description"),
                    success_criteria=data.get("success_criteria", "Task completed"),
                    difficulty=data.get("difficulty", 5)
                )
                
                logger.info(f"Proposed task #{self._task_count}: {task.name}")
                logger.info(f"- Description: {task.description}")
                logger.info(f"- Success: {task.success_criteria}")
                
                # checkj duplicate
                if self._is_duplicate_task(task.name):
                    self._dedup_retries += 1
                    logger.warning(f"Duplicate task detected: '{task.name}' (retry {self._dedup_retries}/3)")
                    if self._dedup_retries >= 3:
                        logger.warning("Max dedup retries reached, accepting task anyway")
                    else:
                        return self.propose_next_task(observation, _is_retry=True)
                
                task = self._fix_success_criteria(task)
                return task
            else:
                logger.warning(f"Could not parse task proposal: {response[:200]}")
                return None
                
        except Exception as e:
            logger.error(f"Task proposal failed: {e}")
            return None
    
    @staticmethod
    def _normalize_task_name(name: str) -> str:
        return name.lower().replace("-", "_").replace(" ", "_")

    @staticmethod
    def _pyramid_stage(task_norm: str) -> Optional[str]:
        """
        Distinguish pyramid sub-stages so we can progress from base -> full pyramid.
        """
        if "pyramid" not in task_norm:
            return None

        has_base = "base" in task_norm
        has_top_or_full = any(
            token in task_norm for token in ("top", "peak", "full", "three", "3", "complete")
        )

        if has_base and not has_top_or_full:
            return "pyramid_base"
        return "pyramid_full"

    def _is_duplicate_task(self, proposed_name: str) -> bool:
        """Check if proposed task is similar to completed task."""
        proposed = self._normalize_task_name(proposed_name)
        proposed_pyramid_stage = self._pyramid_stage(proposed)

        task_keywords = {
            "box": ["box", "container"],
            "stack": ["stack", "tower"],
            "line": ["line", "row"],
            "pick_place": ["pick_and_place", "pick_place"],
            "sort": ["sort", "organize"],
            "clear": ["clear", "move_all"],
        }

        for completed in self._completed_tasks:
            completed_norm = self._normalize_task_name(completed)
            completed_pyramid_stage = self._pyramid_stage(completed_norm)

            if proposed == completed_norm:
                return True

            # Pyramid-aware dedup:
            # - same stage => duplicate
            # - full -> base => duplicate (do not regress)
            # - base -> full => allow (intended progression)
            if proposed_pyramid_stage and completed_pyramid_stage:
                if proposed_pyramid_stage == completed_pyramid_stage:
                    logger.info(
                        f"Duplicate detected: '{proposed}' matches '{completed_norm}' via 'pyramid_stage'"
                    )
                    return True
                if proposed_pyramid_stage == "pyramid_base" and completed_pyramid_stage == "pyramid_full":
                    logger.info(
                        f"Duplicate detected: base pyramid '{proposed}' blocked because full pyramid already completed"
                    )
                    return True
                if proposed_pyramid_stage == "pyramid_full" and completed_pyramid_stage == "pyramid_base":
                    continue

            for category, keywords in task_keywords.items():
                proposed_has = any(kw in proposed for kw in keywords)
                completed_has = any(kw in completed_norm for kw in keywords)
                if proposed_has and completed_has:
                    logger.info(f"Duplicate detected: '{proposed}' matches '{completed_norm}' via '{category}'")
                    return True

        return False
    
    def _fix_success_criteria(self, task: OpenTask) -> OpenTask:
        """Replace unrealistic tolerances with achievable ones."""
        import re
        
        criteria = task.success_criteria
        
        # 0.06m tolerance
        criteria = re.sub(
            r'within\s+(0\.0[1-4])\s*m',
            'within 0.06m',
            criteria,
            flags=re.IGNORECASE
        )
        criteria = re.sub(
            r'tolerance\s+of\s+(0\.0[1-4])',
            'tolerance of 0.06',
            criteria,
            flags=re.IGNORECASE
        )
        
        return OpenTask(
            name=task.name,
            description=task.description,
            success_criteria=criteria,
            difficulty=task.difficulty
        )
    
    def is_done(self) -> bool:
        """Check if curriculum is complete (only if max_tasks reached)"""
        return self._task_count >= self.max_tasks
    
    def get_stats(self) -> Dict[str, Any]:
        return {
            "tasks_proposed": self._task_count,
            "tasks_completed": len(self._completed_tasks),
            "tasks_failed": len(self._failed_tasks),
            "skills_learned": len(self._skills),
            "success_rate": len(self._completed_tasks) / max(1, self._task_count),
        }
