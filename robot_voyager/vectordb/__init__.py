import os
from typing import Optional

from vectordb.interface import VectorDBInterface
from vectordb.enums import VectorDBEnums, DistanceMethodEnums
from vectordb.embedder import SkillEmbedder
from vectordb.providers.qdrant import QdrantDBProvider


class VectorDBProviderFactory:
    
    def __init__(self, base_dir: Optional[str] = None):
        """
        Initialize the factory
        """
        if base_dir is None:
            base_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "..", "assets", "database"
            )
        self.base_dir = base_dir
        
    def get_database_path(self, db_name: str) -> str:
        db_path = os.path.join(self.base_dir, db_name)
        if not os.path.exists(db_path):
            os.makedirs(db_path)
        return db_path

    def create(
        self, 
        provider: str = "QDRANT",
        db_name: str = "skill_vectors",
        distance_method: str = "cosine"
    ) -> VectorDBInterface:
        """
        Create a vector database provider.
        
        Args:
            provider: Provider type (we only used QDRANT)
            db_name: Name for the db
            distance_method: Distance metric ("cosine")
            
        Returns:
            Configured VectorDBInterface
        """
        if provider == VectorDBEnums.QDRANT.value:
            db_path = self.get_database_path(db_name)
            return QdrantDBProvider(
                db_path=db_path,
                distance_method=distance_method,
            )
        
        raise ValueError(f"Unsupported vector DB provider: {provider}")


__all__ = [
    "VectorDBInterface",
    "VectorDBEnums", 
    "DistanceMethodEnums",
    "SkillEmbedder",
    "QdrantDBProvider",
    "VectorDBProviderFactory",
]
