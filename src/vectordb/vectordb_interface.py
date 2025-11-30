from abc import ABC, abstractmethod

class VectorDBInterface(ABC):

    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def create_collection(self, collection_name: str, embedding_size: int, do_reset: bool):
        pass
