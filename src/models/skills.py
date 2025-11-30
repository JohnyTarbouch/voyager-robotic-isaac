from pydantic import BaseModel
from typing import Dict, Any

class RetrievedSkill(BaseModel):
    score: float
    skill_name: str
    description: str
    code: str
    preconditions: Dict[str, Any] | None = None
    effects: Dict[str, Any] | None = None
