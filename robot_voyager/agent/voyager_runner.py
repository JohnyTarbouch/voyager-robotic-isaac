import logging
import time
from pathlib import Path
from typing import Optional

from agent.open_curriculum import OpenEndedCurriculum, OpenTask
from agent.critic import CriticAgent
from agent.planner import Planner, ReflectionResult 
from agent.skill_library import SkillLibrary
from agent.sandbox import run_skill, extract_skill_metadata
from agent.skill_executor import create_skill_context  
from agent.metrics import MetricsLogger
from agent.llm_client import LLMClient, LLMConfig

logger = logging.getLogger(__name__)
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))
from cfg.settings import Config
    
class VoyagerRunner:
    def __init__(
        self,
        robot,
        config: Config = None,
        max_tasks: int = 50,
        console = None
    ):
        self.robot = robot
        self.config = config or Config()
        self.max_tasks = max_tasks
        self._console = console
    
        # Init LLM
        self.llm = LLMClient(LLMConfig(
            model=self.config.llm.model,
            base_url=self.config.llm.base_url,
            api_key=self.config.llm.api_key,
        ))
        
        # Voyager 
        self.curriculum = OpenEndedCurriculum(self.llm, max_tasks=max_tasks)
        self.critic = CriticAgent(self.llm)
        self.planner = Planner(self.llm)  
        
        # Skill library
        self.skills = SkillLibrary(
            root=self.config.agent.skills_root,
            use_vector_db=self.config.agent.use_vector_retrieval,
            embedding_model=self.config.vectordb.embedding_model,
            collection_name=self.config.vectordb.collection_name,
        )
        self.skills.load_all()
        
        # Logger
        log_dir = Path(self.config.agent.log_dir) / time.strftime("%Y%m%d_%H%M%S")
        self.metrics = MetricsLogger(log_dir=str(log_dir))  

        # LLM Logger callbacks
        self._last_extracted_code: Optional[str] = None
        self.llm.set_metrics_callback(self._on_llm_call)
        self.planner.set_code_callback(self._on_code_extracted)
        
        # Stats calc
        self.tasks_completed = 0
        self.tasks_failed = 0
        self.skills_learned = 0

    def _on_llm_call(
        self,
        call_type: str,
        system_prompt: str,
        user_prompt: str,
        response_text: str,
        latency_ms: float,
        temperature: float,
        max_tokens: int,
        prompt_tokens: int = None,
        completion_tokens: int = None,
    ):
        self.metrics.log_llm_call(
            call_type=call_type,
            system_prompt=system_prompt,
            user_prompt=user_prompt,
            response_text=response_text,
            extracted_code=self._last_extracted_code,
            latency_ms=latency_ms,
            temperature=temperature,
            max_tokens=max_tokens,
            prompt_tokens=prompt_tokens,
            completion_tokens=completion_tokens,
        )
        self._last_extracted_code = None

    def _on_code_extracted(self, code: str):
        """
        Callback when code is extracted from LLM response
        """
        self._last_extracted_code = code
        self.metrics.log_code_generated(code)
    
    def _print(self, msg: str, style: str = None):
        if self._console and style:
            self._console.print(f"[{style}]{msg}[/{style}]")
        else:
            print(msg)
    
    def run(self):
        """
        Main learning loop for our voyager
        
        Loop:
        1. LLM proposes a task
        2. LLM generates code to solve it
        3. Execute code
        4. LLM (critic) verifies if it worked
        5. Save skill if successful
        """
        self._print("Starting Voyager autonomous learning", "bold green")
        self._print(f"- Max tasks: {self.max_tasks}", "cyan")
        self._print(f"- Verification: LLM Critic (not hard-coded)", "cyan")
        self._print(f"- Tasks: Open-ended (LLM proposes)", "cyan")
        self._print("")
        
        self.metrics.start_session(
            backend=type(self.robot).__name__,
            llm_model=self.config.llm.model,
            max_attempts=self.config.agent.max_attempts,
        )
        
        try:
            # Init reset
            self.robot.reset()
            
            # Update curriculum with current skills
            skill_names = [s["name"] for s in self.skills.list()]  
            self.curriculum.set_skills(skill_names)
            
            # Main loop
            task_num = 0
            while not self.curriculum.is_done():
                task_num += 1
                
                # Get current obs
                obs = self.robot.get_observation()
                
                # 1. LLM proposes next task
                self._print(f"\n{'='*60}", "blue")
                self._print(f"- Task {task_num}: Asking LLM to propose a task.", "bold cyan")
                
                task = self.curriculum.propose_next_task(obs)
                if task is None:
                    self._print("No more tasks. Ending session.", "yellow")
                    break
                
                self._print(f"Task: {task.name}", "bold white")
                self._print(f"- {task.description}", "white")
                self._print(f"- Success: {task.success_criteria}", "dim")
                
                # Log task start
                self.metrics.start_task(
                    task_name=task.name,
                    task_description=task.description,
                    difficulty=task.difficulty,
                )
                
                # 2. Try to solve task
                success = self._solve_task(task)
                
                # 3. Record result
                if success:
                    self.tasks_completed += 1
                    self.curriculum.add_completed(task.name)
                    self._print(f"Task {task_num} SUCCEEDED!", "bold green")
                else:
                    self.tasks_failed += 1
                    self.curriculum.add_failed(task.name)
                    self._print(f"Task {task_num} FAILED", "bold red")
                
                # Log task end
                self.metrics.end_task(success=success)
                
                # Update skills in curriculum
                skill_names = [s["name"] for s in self.skills.list()]
                self.curriculum.set_skills(skill_names)
                
                # Print progress
                self._print_stats()
            
            # Final summary
            self._print_final_summary()
            
        finally:
            self.metrics.end_session()
    
    def _solve_task(self, task: OpenTask) -> bool:
        # Solve a task using LLM-generated code.
        max_attempts = self.config.agent.max_attempts
        timeout_s = self.config.agent.timeout_s
        
        # Get relevant skills for context
        retrieved = self.skills.retrieve(
            task.description, 
            k=16
        )
        
        last_error: Optional[str] = None
        last_reflection: Optional[ReflectionResult] = None

        for attempt in range(1, max_attempts + 1):
            self._print(f"\n  Attempt {attempt}/{max_attempts}", "cyan")
            
            # Get new observation
            obs = self.robot.get_observation()
            pre_state = dict(obs)
            
            # Start logging
            self.metrics.start_attempt(attempt, pre_state)
            
            # Generate skill code
            self._print("Generating code ", "dim")
            plan = self.planner.propose_skill_code(
                task_name=task.name,
                task_description=f"{task.description}\nSuccess criteria: {task.success_criteria}",
                observation=obs,
                available_skills=retrieved,
                attempt=attempt,
                last_error=last_error,
                reflection=last_reflection,
            )
            
            if not plan.code.strip():
                self._print("No code generated", "yellow")
                last_error = "No code generated"
                last_reflection = None
                self.metrics.end_attempt({}, False)
                continue
            
            # Execute code
            self._print("Executing generated code.", "dim")
            exec_start = time.time()
            skill_context = create_skill_context(self.skills, self.robot)
            result = run_skill(plan.code, self.robot, {}, timeout_s, skill_context)
            exec_time_ms = (time.time() - exec_start) * 1000
            
            # Get post state
            post_state = dict(self.robot.get_observation())
            
            if not result.ok:
                error_msg = result.error or "Unknown execution error"
                self._print(f"Execution error: {error_msg[:100]}", "yellow")
                self.metrics.log_execution_result(False, error_msg, exec_time_ms)
                last_error = error_msg
                last_reflection = self.planner.reflect_on_failure(
                    code=plan.code,
                    error=error_msg,
                    pre_state=pre_state,
                    post_state=post_state,
                )
                self.metrics.end_attempt(post_state, False)
                continue
            
            self.metrics.log_execution_result(True, None, exec_time_ms)
            
            # 4. LLM CRITIC VERIFICATION
            self._print("Critic verifying", "dim")
            critic_result = self.critic.verify(
                task_description=f"{task.description}\nSuccess criteria: {task.success_criteria}",
                pre_state=pre_state,
                post_state=post_state,
                executed_code=plan.code
            )
            
            self._print(f" Critic verdict: {'SUCCESS' if critic_result['success'] else 'FAILED'}", 
                            "green" if critic_result['success'] else "red")
            self._print(f"Reasoning: {critic_result['reasoning'][:100]}...", "dim")
            self._print(f"Confidence: {critic_result['confidence']:.1%}", "dim")
            
            self.metrics.end_attempt(post_state, critic_result['success'])
            
            if critic_result['success']:
                # Save the generated critic skill
                meta = extract_skill_metadata(plan.code)
                meta_name = (meta.get("name") or "").strip()
                skill_name = meta_name or task.name.lower().replace(" ", "_").replace("-", "_")
                skill_path = self.skills.save_generated(skill_name, plan.code) 
                if skill_path:
                    self.skills_learned += 1
                    self._print(f"Skill saved: {skill_name}", "green")
                
                return True
            else:
                last_error = f"Critic failed: {critic_result['reasoning']}"
                last_reflection = self.planner.reflect_on_failure(
                    code=plan.code,
                    error=last_error,
                    pre_state=pre_state,
                    post_state=post_state,
                )
        
        return False
    
    def _print_stats(self):
        total = self.tasks_completed + self.tasks_failed
        rate = self.tasks_completed / total * 100 if total > 0 else 0
        self._print(f"Progress: {self.tasks_completed}/{total} tasks ({rate:.0f}%)", "cyan")
        self._print(f"Skills learned: {self.skills_learned}", "cyan")
    
    def _print_final_summary(self):
        stats = self.curriculum.get_stats()
        
        self._print("\n" + "="*60, "bold blue")
        self._print("VOYAGER SESSION COMPLETE", "bold green")
        self._print("="*60, "bold blue")
        self._print(f"  Tasks proposed:  {stats['tasks_proposed']}", "white")
        self._print(f"  Tasks completed: {stats['tasks_completed']}", "green")
        self._print(f"  Tasks failed:    {stats['tasks_failed']}", "red")
        self._print(f"  Success rate:    {stats['success_rate']:.1%}", "cyan")
        self._print(f"  Skills learned:  {stats['skills_learned']}", "yellow")
        self._print("="*60, "bold blue")


def run_voyager_agent(robot, config: Config = None, max_tasks: int = 10):
    try:
        from rich.console import Console
        console = Console()
    except ImportError:
        console = None
    
    runner = VoyagerRunner(robot, config, max_tasks, console)
    runner.run()
    return runner
