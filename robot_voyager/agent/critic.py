"""
Critic Agent LLM-based task verification,we use an LLM to judge if a task succeeded
based on the state before and after execution.
"""

import logging
from typing import Dict, Any

logger = logging.getLogger(__name__)


CRITIC_SYSTEM_PROMPT = """You are a robot task critic. Your job is to determine if a robot task was completed successfully.

You will be given:
1. The task description (what the robot was supposed to do)
2. The state BEFORE the task (positions of robot and objects)
3. The state AFTER the task (positions of robot and objects)
4. The code that was executed

Based on this information, determine if the task was SUCCESSFUL or FAILED.

IMPORTANT GUIDELINES:
- **Judge by PHYSICAL OUTCOME, not by code return value!**
  The code may return False due to overly strict self-verification, but the task could still
  be physically successful. Always compare BEFORE vs AFTER object positions yourself.
- The task's explicit success criteria are binding. If criteria are not met, return FAILED.
- Position errors up to 0.05m per object are normal due to robot precision limits.
- For multi-object structures (pyramids, towers), allow up to 0.10m tolerance between objects
  because each object can drift ~0.05m independently.
- A pyramid is successful only if:
  1) base cubes are roughly side by side, and
  2) a top cube is clearly elevated above the base cubes.
  Practical elevation rule: top cube z should be at least ~0.03m above both base cubes.
  If all involved cubes remain near table height (z <= 0.04), pyramid/tower is FAILED.
- A tower is successful only if cubes have increasing z values at roughly the same XY.
- Do NOT mark success for partial progress if the final structure was not achieved.
- Look for actual changes in the environment that match the task goal.
- Do NOT blindly trust log messages like "Base cubes are not close enough"; verify using STATE AFTER.
- When uncertain, prefer FAILED over a false SUCCESS.
- In reasoning, cite concrete numeric evidence from STATE AFTER (z values and/or distances).

Output your response in this EXACT JSON format:
{
    "success": true or false,
    "reasoning": "Brief explanation of why the task succeeded or failed",
    "confidence": 0.0 to 1.0
}
"""


class CriticAgent:
    """
    LLM-based critic that verifies task completion
    """
    
    def __init__(self, llm_client):
        """
        Initialize critic with LLM client.
        
        Args:
            llm_client: LLM client for making API calls
        """
        self.llm = llm_client
    
    def verify(
        self,
        task_description: str,
        pre_state: Dict[str, Any],
        post_state: Dict[str, Any],
        executed_code: str = None,
        execution_logs: str = None
    ) -> Dict[str, Any]:
        """
        Verify if a task was completed successfully using LLM.
        
        Args:
            task_description: What the robot was supposed to do
            pre_state: Robot/environment state before execution
            post_state: Robot/environment state after execution
            executed_code: The code that was run (optional)
            execution_logs: Logs from robot.log() during execution (optional)
            
        Returns:
            Dict with keys: success (bool), reasoning (str), confidence (float)
        """
        pre_formatted = self._format_state(pre_state)
        post_formatted = self._format_state(post_state)
        
        # exec logs
        logs_section = ""
        if execution_logs:
            logs_section = f"""
EXECUTION LOGS (from robot.log() calls):
{execution_logs}

NOTE: These logs are from the code's own checks, which may use overly strict tolerances.
Do NOT blindly trust failure messages in the logs. Instead, verify by looking at the actual
STATE AFTER positions above - if the physical outcome matches the task goal, it's a SUCCESS
even if the code's own verification said it failed.
"""                         
        
        user_prompt = f"""TASK: {task_description}

STATE BEFORE:
{pre_formatted}

STATE AFTER:
{post_formatted}

{f'CODE EXECUTED:{chr(10)}{executed_code}' if executed_code else ''}
{logs_section}
Did this task succeed? Analyze the state changes and determine if the goal was achieved.
Respond with JSON only: {{"success": true/false, "reasoning": "", "confidence": 0.0-1.0}}
"""
        
        try:
            response = self.llm.chat(
                messages=[
                    {"role": "system", "content": CRITIC_SYSTEM_PROMPT},
                    {"role": "user", "content": user_prompt},
                ],
                temperature=0.0,
                max_tokens=500,
                call_type="critic_verification",
            )
            
            # Parse JSON response
            import json
            import re
            
            # Extract JSON from response
            json_match = re.search(r'\{[^}]+\}', response, re.DOTALL)
            if json_match:
                result = json.loads(json_match.group())
                
                # Ensure required fields
                return {
                    "success": result.get("success", False),
                    "reasoning": result.get("reasoning", "No reasoning provided"),
                    "confidence": result.get("confidence", 0.5)
                }
            else:
                success = "success" in response.lower() and "fail" not in response.lower()
                return {
                    "success": success,
                    "reasoning": response[:200],
                    "confidence": 0.3
                }
                
        except Exception as e:
            logger.error(f"Critic verification failed: {e}")
            return {
                "success": False,
                "reasoning": f"Verification error: {str(e)}",
                "confidence": 0.0
            }
    
    def _format_state(self, state: Dict[str, Any]) -> str:
        lines = []
        
        if "ee_position" in state:
            ee = state["ee_position"]
            lines.append(f"End-effector position: ({ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f})")
        
        if "gripper_width" in state:
            lines.append(f"Gripper width: {state['gripper_width']:.3f}m")
        
        if "objects" in state:
            lines.append("Objects:")
            for name, info in state["objects"].items():
                if isinstance(info, dict):
                    pos = info.get("position", (0, 0, 0))
                    shape = info.get("shape", "")
                    color = info.get("color", "")
                    label = f" ({color} {shape})" if shape else ""
                    lines.append(f"  - {name}{label}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
                else:
                    pos = info
                    lines.append(f"  - {name}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        
        if "box_position" in state:
            bp = state["box_position"]
            lines.append(f"Box (open-top) center: ({bp[0]:.3f}, {bp[1]:.3f}, {bp[2]:.3f})")
        
        return "\n".join(lines)


def create_critic_verifier(critic: CriticAgent, task_description: str):
    """
    Create a verifier function that uses the critic agent.
    """
    def _verify(robot, pre_state: dict, post_state: dict) -> bool:
        result = critic.verify(task_description, pre_state, post_state)
        logger.info(f"Critic verdict: {result['success']} ({result['reasoning'][:100]}...)")
        return result["success"]
    
    return _verify
