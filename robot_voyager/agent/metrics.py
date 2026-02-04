"""Metrics and logging for the robot voyager agent."""

import json
import logging
import os
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional
import time


@dataclass
class LLMCall:
    timestamp: str
    task_name: str
    attempt: int
    call_type: str  # skill_generation
    
    # Request
    system_prompt: str
    user_prompt: str
    temperature: float
    max_tokens: int
    
    # Response
    response_text: str
    extracted_code: Optional[str]
    
    # Timing
    latency_ms: float
    
    # Token counts 
    prompt_tokens: Optional[int] = None
    completion_tokens: Optional[int] = None
    total_tokens: Optional[int] = None


@dataclass
class AttemptMetrics:
    """Metrics for a single task attempt"""
    attempt_number: int
    timestamp: str
    
    # Code generation
    code_generated: bool
    code_length: int
    
    # Execution
    execution_success: bool
    execution_error: Optional[str]
    execution_time_ms: float
    
    # Verification
    verifier_passed: bool
    
    # State changes
    pre_state: Dict[str, Any]
    post_state: Dict[str, Any]
    
    # LLM calls in this attempt
    llm_calls: List[LLMCall] = field(default_factory=list)


@dataclass
class TaskMetrics:
    """Metrics for a complete task"""
    task_name: str
    task_description: str
    task_difficulty: int
    start_time: str
    end_time: Optional[str] = None
    
    # Results
    success: bool = False
    total_attempts: int = 0
    successful_attempt: Optional[int] = None
    
    # Skill info
    saved_skill_name: Optional[str] = None
    saved_skill_path: Optional[str] = None
    
    # Attempts
    attempts: List[AttemptMetrics] = field(default_factory=list)
    
    # Aggregates (computed)
    total_llm_calls: int = 0
    total_llm_latency_ms: float = 0
    total_execution_time_ms: float = 0


@dataclass
class SessionMetrics:
    """Metrics for an entire agent session"""
    session_id: str
    start_time: str
    backend: str
    llm_model: str
    max_attempts: int
    end_time: Optional[str] = None
    
    # Results
    tasks_attempted: int = 0
    tasks_completed: int = 0
    tasks_failed: int = 0
    
    # Tasks
    tasks: List[TaskMetrics] = field(default_factory=list)
    
    # Aggregates
    total_llm_calls: int = 0
    total_skills_generated: int = 0


class MetricsLogger:
    """Logging"""
    def __init__(self, log_dir: str = "logs"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.log_dir / self.session_id
        self.session_dir.mkdir(parents=True, exist_ok=True)
        
        self.session: Optional[SessionMetrics] = None
        self.current_task: Optional[TaskMetrics] = None
        self.current_attempt: Optional[AttemptMetrics] = None
        
        self._setup_loggers()
    
    def _setup_loggers(self):
        """File loggers"""
        # Main log
        self.main_logger = logging.getLogger("voyager.main")
        main_handler = logging.FileHandler(self.session_dir / "agent.log", encoding="utf-8")
        main_handler.setFormatter(logging.Formatter(
            "%(asctime)s [%(levelname)s] %(message)s"
        ))
        self.main_logger.addHandler(main_handler)
        self.main_logger.setLevel(logging.DEBUG)
        
        # LLM communicaion log
        self.llm_logger = logging.getLogger("voyager.llm")
        llm_handler = logging.FileHandler(self.session_dir / "llm_communications.log", encoding="utf-8")
        llm_handler.setFormatter(logging.Formatter("%(message)s"))
        self.llm_logger.addHandler(llm_handler)
        self.llm_logger.setLevel(logging.DEBUG)
        
        # Metrics log (JSON)
        self.metrics_file = self.session_dir / "metrics.jsonl"

        # Robot/runtime log (captures robot.log via enviroment.* loggers)
        self.robot_log_file = self.session_dir / "robot.log"
        self._attach_file_logger("enviroment.isaac_franka", self.robot_log_file)

    def _attach_file_logger(self, logger_name: str, log_path: Path) -> None:
        """Attach a file handler to a logger if not already present."""
        logger = logging.getLogger(logger_name)
        log_path_str = str(log_path)
        for handler in logger.handlers:
            if isinstance(handler, logging.FileHandler) and getattr(handler, "baseFilename", None) == log_path_str:
                return
        handler = logging.FileHandler(log_path_str, encoding="utf-8")
        handler.setFormatter(logging.Formatter(
            "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
        ))
        logger.addHandler(handler)
        if logger.level == logging.NOTSET:
            logger.setLevel(logging.INFO)
    
    def start_session(self, backend: str, llm_model: str, max_attempts: int):
        """Start a new session."""
        self.session = SessionMetrics(
            session_id=self.session_id,
            start_time=datetime.now().isoformat(),
            backend=backend,
            llm_model=llm_model,
            max_attempts=max_attempts,
        )
        
        self.main_logger.info("="*60)
        self.main_logger.info(f"SESSION STARTED: {self.session_id}")
        self.main_logger.info(f"Backend: {backend}")
        self.main_logger.info(f"LLM Model: {llm_model}")
        self.main_logger.info(f"Max Attempts: {max_attempts}")
        self.main_logger.info("="*60)
        
        self._write_metrics({"event": "session_start", "data": asdict(self.session)})
    
    def end_session(self):
        if self.session:
            self.session.end_time = datetime.now().isoformat()
            
            # Compute aggregates
            for task in self.session.tasks:
                self.session.total_llm_calls += task.total_llm_calls
                if task.success:
                    self.session.total_skills_generated += 1
            
            self.main_logger.info("="*60)
            self.main_logger.info("SESSION ENDED")
            self.main_logger.info(f"Tasks Attempted: {self.session.tasks_attempted}")
            self.main_logger.info(f"Tasks Completed: {self.session.tasks_completed}")
            self.main_logger.info(f"Tasks Failed: {self.session.tasks_failed}")
            self.main_logger.info(f"Total LLM Calls: {self.session.total_llm_calls}")
            self.main_logger.info(f"Skills Generated: {self.session.total_skills_generated}")
            self.main_logger.info("="*60)
            
            # Save full session metrics
            self._save_session_summary()
            self._write_metrics({"event": "session_end", "data": asdict(self.session)})
    
    def start_task(self, task_name: str, task_description: str, difficulty: int = 1):
        """Tracking a new task"""
        self.current_task = TaskMetrics(
            task_name=task_name,
            task_description=task_description,
            task_difficulty=difficulty,
            start_time=datetime.now().isoformat(),
        )
        
        self.main_logger.info("-"*60)
        self.main_logger.info(f"TASK STARTED: {task_name}")
        self.main_logger.info(f"Description: {task_description}")
        self.main_logger.info("-"*60)
        
        self._write_metrics({"event": "task_start", "task": task_name})
    
    def end_task(self, success: bool, skill_name: str = None, skill_path: str = None):
        if self.current_task:
            self.current_task.end_time = datetime.now().isoformat()
            self.current_task.success = success
            self.current_task.saved_skill_name = skill_name
            self.current_task.saved_skill_path = skill_path
            
            # Compute aggregates
            for attempt in self.current_task.attempts:
                self.current_task.total_execution_time_ms += attempt.execution_time_ms
                for llm_call in attempt.llm_calls:
                    self.current_task.total_llm_calls += 1
                    self.current_task.total_llm_latency_ms += llm_call.latency_ms
            
            self.main_logger.info("-"*60)
            self.main_logger.info(f"TASK ENDED: {self.current_task.task_name}")
            self.main_logger.info(f"Success: {success}")
            self.main_logger.info(f"Total Attempts: {self.current_task.total_attempts}")
            self.main_logger.info(f"Total LLM Calls: {self.current_task.total_llm_calls}")
            self.main_logger.info(f"Total LLM Latency: {self.current_task.total_llm_latency_ms:.0f}ms")
            if skill_name:
                self.main_logger.info(f"Saved Skill: {skill_name}")
            self.main_logger.info("-"*60)
            
            # Add to session
            if self.session:
                self.session.tasks.append(self.current_task)
                self.session.tasks_attempted += 1
                if success:
                    self.session.tasks_completed += 1
                else:
                    self.session.tasks_failed += 1
            
            self._write_metrics({
                "event": "task_end",
                "task": self.current_task.task_name,
                "success": success,
                "attempts": self.current_task.total_attempts,
            })
            
            self.current_task = None
    
    def start_attempt(self, attempt_number: int, pre_state: Dict[str, Any]):
        """Start a new attempt"""
        self.current_attempt = AttemptMetrics(
            attempt_number=attempt_number,
            timestamp=datetime.now().isoformat(),
            code_generated=False,
            code_length=0,
            execution_success=False,
            execution_error=None,
            execution_time_ms=0,
            verifier_passed=False,
            pre_state=pre_state,
            post_state={},
        )
        
        self.main_logger.info(f"  Attempt {attempt_number} started")
        self._write_metrics({
            "event": "attempt_start",
            "task": self.current_task.task_name if self.current_task else "unknown",
            "attempt": attempt_number,
        })
    
    def end_attempt(self, post_state: Dict[str, Any], verifier_passed: bool):
        if self.current_attempt:
            self.current_attempt.post_state = post_state
            self.current_attempt.verifier_passed = verifier_passed
            
            self.main_logger.info(f" Attempt {self.current_attempt.attempt_number} ended")
            self.main_logger.info(f"- Execution Success: {self.current_attempt.execution_success}")
            self.main_logger.info(f"- Verifier Passed: {verifier_passed}")
            
            if self.current_task:
                self.current_task.attempts.append(self.current_attempt)
                self.current_task.total_attempts += 1
                if verifier_passed:
                    self.current_task.successful_attempt = self.current_attempt.attempt_number
            
            self._write_metrics({
                "event": "attempt_end",
                "task": self.current_task.task_name if self.current_task else "unknown",
                "attempt": self.current_attempt.attempt_number,
                "verifier_passed": verifier_passed,
            })
            
            self.current_attempt = None
    
    def log_llm_call(
        self,
        call_type: str,
        system_prompt: str,
        user_prompt: str,
        response_text: str,
        extracted_code: Optional[str],
        latency_ms: float,
        temperature: float = 0.2,
        max_tokens: int = 1024,
        prompt_tokens: int = None,
        completion_tokens: int = None,
    ):
        """Log an LLM API call"""
        llm_call = LLMCall(
            timestamp=datetime.now().isoformat(),
            task_name=self.current_task.task_name if self.current_task else "unknown",
            attempt=self.current_attempt.attempt_number if self.current_attempt else 0,
            call_type=call_type,
            system_prompt=system_prompt,
            user_prompt=user_prompt,
            temperature=temperature,
            max_tokens=max_tokens,
            response_text=response_text,
            extracted_code=extracted_code,
            latency_ms=latency_ms,
            prompt_tokens=prompt_tokens,
            completion_tokens=completion_tokens,
            total_tokens=(prompt_tokens + completion_tokens) if prompt_tokens and completion_tokens else None,
        )
        
        if self.current_attempt:
            self.current_attempt.llm_calls.append(llm_call)
        
        # Log to LLM communications file
        self.llm_logger.info("="*80)
        self.llm_logger.info(f"TIMESTAMP: {llm_call.timestamp}")
        self.llm_logger.info(f"TASK: {llm_call.task_name} | ATTEMPT: {llm_call.attempt} | TYPE: {call_type}")
        self.llm_logger.info(f"LATENCY: {latency_ms:.0f}ms")
        self.llm_logger.info("-"*80)
        self.llm_logger.info("SYSTEM PROMPT:")
        self.llm_logger.info(system_prompt)
        self.llm_logger.info("-"*80)
        self.llm_logger.info("USER PROMPT:")
        self.llm_logger.info(user_prompt)
        self.llm_logger.info("-"*80)
        self.llm_logger.info("RESPONSE:")
        self.llm_logger.info(response_text)
        if extracted_code:
            self.llm_logger.info("-"*80)
            self.llm_logger.info("EXTRACTED CODE:")
            self.llm_logger.info(extracted_code)
        self.llm_logger.info("="*80)
        self.llm_logger.info("")
        
        self.main_logger.info(f"    LLM Call ({call_type}): {latency_ms:.0f}ms")
    
    def log_code_generated(self, code: str):
        if self.current_attempt:
            self.current_attempt.code_generated = bool(code)
            self.current_attempt.code_length = len(code) if code else 0
    
    def log_execution_result(self, success: bool, error: str = None, execution_time_ms: float = 0):
        if self.current_attempt:
            self.current_attempt.execution_success = success
            self.current_attempt.execution_error = error
            self.current_attempt.execution_time_ms = execution_time_ms
        
        self.main_logger.info(f"    Execution: {'Success' if success else 'Failed'}")
        if error:
            self.main_logger.info(f"    Error: {error[:200]}...")
    
    def _write_metrics(self, data: Dict[str, Any]):
        data["timestamp"] = datetime.now().isoformat()
        with open(self.metrics_file, "a", encoding="utf-8") as f:
            f.write(json.dumps(data, default=str) + "\n")
    
    def _save_session_summary(self):
        summary_file = self.session_dir / "session_summary.json"
        with open(summary_file, "w", encoding="utf-8") as f:
            json.dump(asdict(self.session), f, indent=2, default=str)
        
        report_file = self.session_dir / "session_report.txt"
        with open(report_file, "w", encoding="utf-8") as f:
            f.write("="*60 + "\n")
            f.write("ROBOT VOYAGER SESSION REPORT\n")
            f.write("="*60 + "\n\n")
            
            f.write(f"Session ID: {self.session.session_id}\n")
            f.write(f"Start Time: {self.session.start_time}\n")
            f.write(f"End Time: {self.session.end_time}\n")
            f.write(f"Backend: {self.session.backend}\n")
            f.write(f"LLM Model: {self.session.llm_model}\n\n")
            
            f.write("-"*60 + "\n")
            f.write("SUMMARY\n")
            f.write("-"*60 + "\n")
            f.write(f"Tasks Attempted: {self.session.tasks_attempted}\n")
            f.write(f"Tasks Completed: {self.session.tasks_completed}\n")
            f.write(f"Tasks Failed: {self.session.tasks_failed}\n")
            success_rate = (self.session.tasks_completed / self.session.tasks_attempted * 100) if self.session.tasks_attempted > 0 else 0
            f.write(f"Success Rate: {success_rate:.1f}%\n")
            f.write(f"Total LLM Calls: {self.session.total_llm_calls}\n")
            f.write(f"Skills Generated: {self.session.total_skills_generated}\n\n")
            
            f.write("-"*60 + "\n")
            f.write("TASK DETAILS\n")
            f.write("-"*60 + "\n\n")
            
            for task in self.session.tasks:
                f.write(f"Task: {task.task_name}\n")
                f.write(f"- Description: {task.task_description}\n")
                f.write(f"- Success: {task.success}\n")
                f.write(f"- Attempts: {task.total_attempts}\n")
                f.write(f"- LLM Calls: {task.total_llm_calls}\n")
                f.write(f"- LLM Latency: {task.total_llm_latency_ms:.0f}ms\n")
                if task.saved_skill_name:
                    f.write(f"  Saved Skill: {task.saved_skill_name}\n")
                f.write("\n")
        
        self.main_logger.info(f"Session summary saved to: {summary_file}")
        self.main_logger.info(f"Session report saved to: {report_file}")
    
    def get_session_dir(self) -> Path:
        """Get the session log directory."""
        return self.session_dir
