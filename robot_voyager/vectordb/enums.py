from enum import Enum


class VectorDBEnums(Enum):
    QDRANT = "QDRANT"
    CHROMA = "CHROMA"


class DistanceMethodEnums(Enum):
    COSINE = "cosine"
    DOT = "dot"
    EUCLIDEAN = "euclidean"
