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

Your current skill library:
{skills}

Tasks you have already completed:
{completed_tasks}

Tasks that failed (avoid repeating these exact tasks):
{failed_tasks}

GUIDELINES FOR PROPOSING TASKS:
1. Start simple, gradually increase complexity
2. Build on skills you've already learned
3. Be CREATIVE! The environment has 4 colored cubes - explore possibilities!
CRITICAL - SUCCESS CRITERIA TOLERANCES:
- Robot has ~0.05m position offset - NEVER use tolerances < 0.05m!
- For position verification, use tolerance of 0.05-0.08m (NOT SMALLER!)
- Example good criteria: "cube within 0.06m of target position"
- Example BAD criteria: "cube within 0.01m of target" (impossible!)
4. Task ideas with 4 cubes:
   - First start with easy tasks: pick-and-place one cube, reach target point
   - Then combine skills for more complex tasks.
   - Pyramids: 2 base + 1 top, implement as a first complex task
   - Stacking: towers of 2, 3, or 4 cubes
   - Patterns: lines, squares, L-shapes, T-shapes
   - Sorting: arrange by position or group colors
   - Moving: relocate cubes to specific positions
   - Clearing: move all cubes to one side
5. Challenge yourself but stay within robot capabilities:
   - Robot can reach: x: 0.3-0.7, y: -0.4-0.4, z: 0.0-0.6
   - Gripper can grasp objects ~0.05m wide
   - Cubes are 0.05m x 0.05m x 0.05m


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
    
    def set_skills(self, skills: List[str]):
        """Update the list of learned skills"""
        self._skills = skills
    
    def add_completed(self, task_name: str):
        """Record completed task"""
        self._completed_tasks.append(task_name)
    
    def add_failed(self, task_name: str):
        """Record failed task"""
        self._failed_tasks.append(task_name)
    
    def propose_next_task(self, observation: Dict[str, Any]) -> Optional[OpenTask]:
        """
        Use LLM to propose the next task based on current state
        """
        self._task_count += 1
        
        # limit
        if self._task_count > self.max_tasks:
            logger.info(f"Reached max tasks limit ({self.max_tasks})")
            return None
        
        objects_str = "None detected"
        if "objects" in observation:
            objects_list = []
            for name, pos in observation["objects"].items():
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
                task = self._fix_success_criteria(task)
                return task
            else:
                logger.warning(f"Could not parse task proposal: {response[:200]}")
                return None
                
        except Exception as e:
            logger.error(f"Task proposal failed: {e}")
            return None
    
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