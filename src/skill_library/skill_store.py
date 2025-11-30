from src.common.config import (
    VECTOR_DB_PROVIDER,
    VECTOR_DB_PATH,
    VECTOR_DB_DISTANCE_METHOD,
    SKILLS_COLLECTION_NAME,
    SKILL_RETRIEVAL_TOP_K,
    SKILL_RETRIEVAL_SCORE_THRESHOLD,
    SKILL_EMBEDDING_MODEL,
)
from src.vectordb import VectorDBProviderFactory, VectorDBEnums
from src.vectordb.skill_embedding import SkillEmbedder


class SkillStore:
    def __init__(self):
        self.embedder = SkillEmbedder(SKILL_EMBEDDING_MODEL)

        class _Config:
            VECTOR_DB_PATH = VECTOR_DB_PATH
            VECTOR_DB_DISTANCE_METHOD = VECTOR_DB_DISTANCE_METHOD

        factory = VectorDBProviderFactory(_Config())
        self.db = factory.create(VectorDBEnums[VECTOR_DB_PROVIDER].value)
        self.db.connect()

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
