from sentence_transformers import SentenceTransformer

class SkillEmbedder:
    def __init__(self, model_name: str):
        self.model = SentenceTransformer(model_name)
        self.dim = self.model.get_sentence_embedding_dimension()
    
    def embed(self, text: str) -> list:
        return self.model.encode(text, normalize_embeddings=True).tolist()
