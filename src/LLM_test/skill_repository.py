from dataclasses import dataclass
from typing import List, Optional

from logger import get_skills_logger
from config import (
    SKILLS_COLLECTION_NAME,
    SKILL_RETRIEVAL_TOP_K,
    SKILL_RETRIEVAL_SCORE_THRESHOLD,
    SKILL_EMBEDDING_MODEL,
    VECTOR_DB_PROVIDER,
)
from skill_embedding import SkillEmbedder
from vectordb_providor import VectorDBProviderFactory


@dataclass
class SkillSpec:
    name: str
    description: str
    code: str
    preconditions: Optional[dict] = None
    effects: Optional[dict] = None


class SkillRepository:
    def __init__(self, config_module):
        self.logger = get_skills_logger()
        self.config = config_module

        # Embeddings
        self.embedder = SkillEmbedder(SKILL_EMBEDDING_MODEL)

        # Vector DB (Qdrant)
        factory = VectorDBProviderFactory(config_module)
        self.db = factory.create(VECTOR_DB_PROVIDER)
        self.db.connect()
        self.db.create_collection(
            collection_name=SKILLS_COLLECTION_NAME,
            embedding_size=self.embedder.dim,
            do_reset=False,
        )
        self.logger.info(
            f"[SkillRepository] Connected to vector DB provider={VECTOR_DB_PROVIDER}, "
            f"collection={SKILLS_COLLECTION_NAME}, dim={self.embedder.dim}"
        )

    def close(self):
        try:
            self.db.disconnect()
        except Exception:
            pass



    def add_skill(self, spec: SkillSpec) -> bool:
        """
        Insert a new skill into the collection.
        Embedding text = name + description + code (Voyager-style).
        """
        text_for_embedding = f"{spec.name}\n{spec.description}\n{spec.code}"
        embedding = self.embedder.embed(text_for_embedding)

        ok = self.db.insert_skill(
            collection_name=SKILLS_COLLECTION_NAME,
            embedding=embedding,
            skill_name=spec.name,
            description=spec.description,
            code=spec.code,
            preconditions=spec.preconditions,
            effects=spec.effects,
        )

        if ok:
            self.logger.info(f"[SkillRepository] Stored skill: {spec.name}")
        else:
            self.logger.error(f"[SkillRepository] Failed to store skill: {spec.name}")

        return ok



    def retrieve(self, query: str) -> List[SkillSpec]:
        """
        Retrieve the most relevant skills for a given task description.
        """
        embedding = self.embedder.embed(query)
        retrieved = self.db.search_skills(
            collection_name=SKILLS_COLLECTION_NAME,
            embedding=embedding,
            limit=SKILL_RETRIEVAL_TOP_K,
        )

        skills: List[SkillSpec] = []
        for s in retrieved:
            if s.score < SKILL_RETRIEVAL_SCORE_THRESHOLD:
                continue
            skills.append(
                SkillSpec(
                    name=s.skill_name,
                    description=s.description,
                    code=s.code,
                    preconditions=s.preconditions,
                    effects=s.effects,
                )
            )

        self.logger.info(
            f"[SkillRepository] Retrieved {len(skills)} skills for query='{query}'"
        )
        return skills
