from src.common.config import (
    SKILLS_COLLECTION_NAME,
    SKILL_RETRIEVAL_TOP_K,
    SKILL_RETRIEVAL_SCORE_THRESHOLD,
    SKILL_EMBEDDING_MODEL,
)
from src.vectordb.skill_embedding import SkillEmbedder


class SkillStore:
    def __init__(self, db):
        self.db = db
        self.embedder = SkillEmbedder(SKILL_EMBEDDING_MODEL)
        self.collection = SKILLS_COLLECTION_NAME
        
        
        self.db.create_collection(
            collection_name=self.collection,
            embedding_size=self.embedder.dim,
            do_reset=False,
        )

    def retrieve(self, query: str):
        query_embedding = self.embedder.embed(query)

        skills = self.db.search_skills(
            collection_name=self.collection,
            embedding=query_embedding,
            limit=SKILL_RETRIEVAL_TOP_K,
        )

        # Score-based filter
        return [
            s for s in skills
            if s.score >= SKILL_RETRIEVAL_SCORE_THRESHOLD
        ]

    def add_skill(
        self,
        skill_name: str,
        description: str,
        code: str,
        preconditions: dict | None = None,
        effects: dict | None = None,
    ) -> bool:
        embedding = self.embedder.embed(description)

        return self.db.insert_skill(
            collection_name=self.collection,
            embedding=embedding,
            skill_name=skill_name,
            description=description,
            code=code,
            preconditions=preconditions,
            effects=effects,
        )

    def close(self):
            self.db.disconnect()
