
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass
class Skill:
    name: str
    description: str
    tags: List[str]
    code: str
    path: Optional[Path] = None
    preconditions: Optional[Dict[str, Any]] = None
    effects: Optional[Dict[str, Any]] = None


@dataclass
class RetrievedSkill:
    skill_name: str
    description: str
    code: str
    score: float = 0.0
    preconditions: Optional[Dict[str, Any]] = None
    effects: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.skill_name,
            "description": self.description,
            "code": self.code,
            "score": self.score,
            "preconditions": self.preconditions,
            "effects": self.effects,
        }


@dataclass
class TaskResult:
    success: bool
    skill_code: str
    error: Optional[str] = None
    attempts: int = 0
    saved_path: Optional[Path] = None


@dataclass 
class ExecutionResult:
    ok: bool
    returned: Any = None
    error: Optional[str] = None
