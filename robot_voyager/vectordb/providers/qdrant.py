import hashlib
import logging
from dataclasses import dataclass
from typing import List, Optional, Dict, Any

from vectordb.interface import VectorDBInterface, RetrievedSkill
from vectordb.enums import DistanceMethodEnums

logger = logging.getLogger(__name__)


class QdrantDBProvider(VectorDBInterface):
    """Qdrant-based vector database for skill storage and retrieval."""

    def __init__(self, db_path: str, distance_method: str = "cosine"):
        try:
            from qdrant_client import QdrantClient, models
            self._QdrantClient = QdrantClient
            self._models = models
        except ImportError:
            raise ImportError(
                "qdrant-client package required. "
                "Install with: pip install qdrant-client"
            )

        self.client: Optional[QdrantClient] = None
        self.db_path = db_path
        self.logger = logging.getLogger(__name__)

        # Set metric
        if distance_method == DistanceMethodEnums.COSINE.value:
            self.distance = self._models.Distance.COSINE
        elif distance_method == DistanceMethodEnums.DOT.value:
            self.distance = self._models.Distance.DOT
        else:
            raise ValueError(f"Invalid distance method: {distance_method}")

    def connect(self) -> None:
        """Connect to Qdrant db"""
        self.client = self._QdrantClient(path=self.db_path)
        self.logger.info(f"Connected to Qdrant at {self.db_path}")

    def disconnect(self) -> None:
        try:
            if self.client:
                self.client.close()
        except Exception:
            pass
        self.client = None

    def is_collection_existed(self, collection_name: str) -> bool:
        if not self.client:
            raise RuntimeError("Not connected. Call connect() first.")
        return self.client.collection_exists(collection_name=collection_name)

    def list_all_collections(self) -> List:
        if not self.client:
            raise RuntimeError("Not connected. Call connect() first.")
        return self.client.get_collections()

    def get_collection_info(self, collection_name: str) -> dict:
        if not self.client:
            raise RuntimeError("Not connected. Call connect() first.")
        return self.client.get_collection(collection_name=collection_name)

    def delete_collection(self, collection_name: str) -> bool:
        if not self.client:
            raise RuntimeError("Not connected. Call connect() first.")
        if self.is_collection_existed(collection_name):
            return self.client.delete_collection(collection_name=collection_name)
        return False

    def create_collection(
        self, 
        collection_name: str, 
        embedding_size: int, 
        do_reset: bool = False
    ) -> bool:
        """
        Create a new collection.
        
        Args:
            collection_name: Name for the collection
            embedding_size: Dimension of embedding vectors
            do_reset: If True, delete existing collection first
            
        Returns:
            True if collection created
        """
        if not self.client:
            raise RuntimeError("Not connected. Call connect() first")
            
        if do_reset:
            self.delete_collection(collection_name)

        if not self.is_collection_existed(collection_name):
            self.client.create_collection(
                collection_name=collection_name,
                vectors_config=self._models.VectorParams(
                    size=embedding_size,
                    distance=self.distance
                )
            )
            self.logger.info(f"Created collection '{collection_name}' with dim={embedding_size}")
            return True

        return False

    def _skill_id(self, skill_name: str) -> int:
        return int(hashlib.sha256(skill_name.encode()).hexdigest(), 16) % (2**63)

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
        """
        Insert skill in the vector db
        
        Args:
            collection_name: Target collection
            embedding: Vector embedding of the skill
            skill_name: Unique name for the skill
            description: Readable description
            code: The Python code implementing the skill
            preconditions: Optional preconditions for the skill
            effects: Optional effects of executing the skill
        """
        if not self.client:
            raise RuntimeError("Not connected. Call connect() first.")
            
        if not self.is_collection_existed(collection_name):
            self.logger.error(f"Collection does not exist: {collection_name}")
            return False

        try:
            self.client.upsert(
                collection_name=collection_name,
                points=[
                    self._models.PointStruct(
                        id=self._skill_id(skill_name),
                        vector=embedding,
                        payload={
                            "skill_name": skill_name,
                            "description": description,
                            "code": code,
                            "preconditions": preconditions or {},
                            "effects": effects or {},
                        },
                    )
                ],
            )
            self.logger.info(f"Inserted skill: {skill_name}")
            return True

        except Exception as e:
            self.logger.error(f"Error inserting skill '{skill_name}': {e}")
            return False

    def search_skills(
        self,
        collection_name: str,
        embedding: List[float],
        limit: int = 5,
    ) -> List[RetrievedSkill]:
        """
        Search for similar skills/query using vector similarity.
        """
        if not self.client:
            raise RuntimeError("Not connected.")
            
        if not self.is_collection_existed(collection_name):
            self.logger.warning(f"Collection does not exist: {collection_name}")
            return []

        try:
            raw = self.client.query_points(
                collection_name=collection_name,
                query=embedding,
                limit=limit,
            )
        except Exception as e:
            self.logger.error(f"Search error: {e}")
            return []

        results = []
        for pt in raw.points:
            results.append(
                RetrievedSkill(
                    score=pt.score,
                    skill_name=pt.payload["skill_name"],
                    description=pt.payload["description"],
                    code=pt.payload["code"],
                    preconditions=pt.payload.get("preconditions"),
                    effects=pt.payload.get("effects"),
                )
            )
        return results

    def get_all_skills(self, collection_name: str, limit: int = 100) -> List[RetrievedSkill]:
        """
        Retrieve all skills from a collection.
        """
        if not self.client:
            raise RuntimeError("Not connected. Call connect() first.")
            
        if not self.is_collection_existed(collection_name):
            return []

        try:
            results = self.client.scroll(
                collection_name=collection_name,
                limit=limit,
                with_payload=True,
                with_vectors=False,
            )
            
            skills = []
            for pt in results[0]:  # results is (points, next_page_offset)
                skills.append(
                    RetrievedSkill(
                        score=1.0,
                        skill_name=pt.payload["skill_name"],
                        description=pt.payload["description"],
                        code=pt.payload["code"],
                        preconditions=pt.payload.get("preconditions"),
                        effects=pt.payload.get("effects"),
                    )
                )
            return skills
            
        except Exception as e:
            self.logger.error(f"Error getting all skills: {e}")
            return []
