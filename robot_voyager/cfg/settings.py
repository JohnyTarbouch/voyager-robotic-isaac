"""Config settings for robot voyager"""

import os
from dataclasses import dataclass, field
from typing import Optional


def _load_env_vars():
    if os.path.exists(".env"):
        for line in open(".env", "r", encoding="utf-8"):
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip())


_load_env_vars()


@dataclass
class LLMConfig:
    base_url: str = field(default_factory=lambda: os.getenv("LLM_BASE_URL", "").strip())
    api_key: str = field(default_factory=lambda: os.getenv("LLM_API_KEY", "none").strip())
    model: str = field(default_factory=lambda: os.getenv("LLM_MODEL", "llama-3.3-70b-instruct"))


@dataclass
class VectorDBConfig:
    provider: str = "QDRANT"
    db_path: str = "skill_vectors"
    distance_method: str = "cosine"
    collection_name: str = "skills"
    embedding_model: str = "all-MiniLM-L6-v2"


@dataclass
class AgentConfig:
    max_attempts: int = field(default_factory=lambda: int(os.getenv("AGENT_MAX_ATTEMPTS", "3")))
    timeout_s: int = field(default_factory=lambda: int(os.getenv("AGENT_TIMEOUT_S", "25")))
    skills_root: str = "skills"
    log_dir: str = field(default_factory=lambda: os.getenv("AGENT_LOG_DIR", "logs"))
    use_vector_retrieval: bool = True 
    autonomous_curriculum: bool = field(default_factory=lambda: os.getenv("AGENT_AUTONOMOUS", "").lower() == "true")
    max_autonomous_tasks: int = field(default_factory=lambda: int(os.getenv("AGENT_MAX_TASKS", "20")))


@dataclass
class Config:
    llm: LLMConfig = field(default_factory=LLMConfig)
    vectordb: VectorDBConfig = field(default_factory=VectorDBConfig)
    agent: AgentConfig = field(default_factory=AgentConfig)
    
    @property
    def VECTOR_DB_PATH(self) -> str:
        return self.vectordb.db_path
    
    @property
    def VECTOR_DB_DISTANCE_METHOD(self) -> str:
        return self.vectordb.distance_method


default_config = Config()
