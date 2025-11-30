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

    def close(self):
            self.db.disconnect()
