from typing import List, Optional
import logging

logger = logging.getLogger(__name__)


class SkillEmbedder:
    """Generates embeddings for skill descriptions and queries"""
    
    def __init__(self, model_name: str = "all-MiniLM-L6-v2"):
        self.model_name = model_name
        self._model = None
        self._dim: Optional[int] = None
    
    def _load_model(self):
        if self._model is None:
            try:
                from sentence_transformers import SentenceTransformer
                logger.info(f"Loading embedding model: {self.model_name}")
                self._model = SentenceTransformer(self.model_name)
                self._dim = self._model.get_sentence_embedding_dimension()
                logger.info(f"Model loaded. Embedding dimension: {self._dim}")
            except ImportError:
                raise ImportError(
                    "sentence-transformers package required. "
                    "Install with: pip install sentence-transformers"
                )
    
    @property
    def dim(self) -> int:
        """Get the embedding dim"""
        self._load_model()
        return self._dim
    
    def embed(self, text: str) -> List[float]:
        """
        Generate embedding for a single text string
        
        Args:
            text: Skill description or query
            
        Returns:
            List of floats representing the embedding vector
        """
        self._load_model()
        embedding = self._model.encode(text, normalize_embeddings=True)
        return embedding.tolist()
    
    def embed_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts efficiently
        """
        self._load_model()
        embeddings = self._model.encode(texts, normalize_embeddings=True)
        return [emb.tolist() for emb in embeddings]
    
    def embed_skill(self, name: str, description: str, tags: List[str]) -> List[float]:
        """
        Generate embedding for a skill using its metadata
        
        Args:
            name: Skill name
            description: Skill description
            tags: List of skill tags
            
        Returns:
            Embedding vector for the skill
        """
        # Combine skill metadata into a single text for embedding
        combined = f"{name}. {description}. Tags: {', '.join(tags)}"
        return self.embed(combined)
