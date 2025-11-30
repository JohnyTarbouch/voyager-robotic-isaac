import hashlib
from qdrant_client import QdrantClient, models
import logging
from typing import List
from src.models.skills import RetrievedSkill
from src.vectordb.vectordb_interface import VectorDBInterface
from src.vectordb.vectoredb_enums import DistanceMethodEnums

class QdrantDBProvider(VectorDBInterface):

    def __init__(self, db_path: str, distance_method: str):

        self.client = None
        self.db_path = db_path
        self.logger = logging.getLogger(__name__)

        if distance_method == DistanceMethodEnums.COSINE.value:
            self.distance = models.Distance.COSINE
        elif distance_method == DistanceMethodEnums.DOT.value:
            self.distance = models.Distance.DOT
        else:
            raise ValueError("Invalid distance method")

    def connect(self):
        self.client = QdrantClient(path=self.db_path)

    def disconnect(self):
        try:
            if self.client:
                self.client.close()
        except Exception:
            pass
        self.client = None


    def is_collection_existed(self, collection_name: str) -> bool:
        return self.client.collection_exists(collection_name=collection_name)
    
    def list_all_collections(self) -> List:
        return self.client.get_collections()
    
    def get_collection_info(self, collection_name: str) -> dict:
        return self.client.get_collection(collection_name=collection_name)
    
    def delete_collection(self, collection_name: str):
        if self.is_collection_existed(collection_name):
            return self.client.delete_collection(collection_name=collection_name)
        
    def create_collection(self, collection_name: str, 
                                embedding_size: int,
                                do_reset: bool = False):
        if do_reset:
            _ = self.delete_collection(collection_name=collection_name)
        
        if not self.is_collection_existed(collection_name):
            _ = self.client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=embedding_size,
                    distance=self.distance
                )
            )

            return True
        
        return False
    
    def _skill_id(self, skill_name: str) -> int:
        return int(hashlib.sha256(skill_name.encode()).hexdigest(), 16) % (2**63)

    def insert_skill(
        self,
        collection_name: str,
        embedding: list,
        skill_name: str,
        description: str,
        code: str,
        preconditions: dict = None,
        effects: dict = None,
    ) -> bool:
        
        if not self.is_collection_existed(collection_name):
            self.logger.error(f"Can not insert new record to non-existed collection: {collection_name}")
            return False
        
        try:
            _ = self.client.upsert(
                collection_name=collection_name,
                points=[
                    models.PointStruct(
                        id=self._skill_id(skill_name),
                        vector=embedding,
                        payload={
                            "skill_name": skill_name,
                            "description": description,
                            "code": code,
                            "preconditions": preconditions,
                            "effects": effects,
                        },
                    )
                ],
            )

        except Exception as e:
            self.logger.error(f"Error while inserting batch: {e}")
            return False

        return True
    
    def insert_many(self, 
                    collection_name: str, texts: list, 
                    vectors: list, metadata: list = None, 
                    record_ids: list = None, batch_size: int = 50):
        
        if metadata is None:
            metadata = [None] * len(texts)

        if record_ids is None:
            record_ids = list(range(0, len(texts)))

        for i in range(0, len(texts), batch_size):
            batch_end = i + batch_size

            batch_texts = texts[i:batch_end]
            batch_vectors = vectors[i:batch_end]
            batch_metadata = metadata[i:batch_end]
            batch_record_ids = record_ids[i:batch_end]

            batch_records = [
                models.Record(
                    id=batch_record_ids[x],
                    vector=batch_vectors[x],
                    payload={
                        "text": batch_texts[x], "metadata": batch_metadata[x]
                    }
                )

                for x in range(len(batch_texts))
            ]

            try:
                _ = self.client.upload_records(
                    collection_name=collection_name,
                    records=batch_records,
                )
            except Exception as e:
                self.logger.error(f"Error while inserting batch: {e}")
                return False

        return True
        
    def search_skills(
        self,
        collection_name: str,
        embedding: list,
        limit: int = 5,
    ):
        try:
            raw = self.client.query_points(
                collection_name=collection_name,
                query=embedding,
                limit=limit,
            )
        except Exception as e:
            self.logger.error(f"Search error: {e}")
            return []

        return [
            RetrievedSkill(
                score=pt.score,
                skill_name=pt.payload["skill_name"],
                description=pt.payload["description"],
                code=pt.payload["code"],
                preconditions=pt.payload.get("preconditions"),
                effects=pt.payload.get("effects"),
            )
            for pt in raw.points
        ]
