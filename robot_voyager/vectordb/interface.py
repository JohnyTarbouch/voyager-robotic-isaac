from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Dict, Any


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


class VectorDBInterface(ABC):

    @abstractmethod
    def connect(self) -> None:
        pass

    @abstractmethod
    def disconnect(self) -> None:
        pass

    @abstractmethod
    def create_collection(
        self, 
        collection_name: str, 
        embedding_size: int, 
        do_reset: bool = False
    ) -> bool:
        pass

    @abstractmethod
    def delete_collection(self, collection_name: str) -> bool:
        pass

    @abstractmethod
    def is_collection_existed(self, collection_name: str) -> bool:
        pass

    @abstractmethod
    def insert_skill(
        self,
        collection_name: str,
        embedding: List[float],
        skill_name: str,
        description: str,
        code: str,
        preconditions: Optional[Dict[str, Any]] = None,
        effects: Optional[Dict[str, Any]] = None,
    ) -> bool:
        pass

    @abstractmethod
    def search_skills(
        self,
        collection_name: str,
        embedding: List[float],
        limit: int = 5,
    ) -> List[RetrievedSkill]:
        pass
