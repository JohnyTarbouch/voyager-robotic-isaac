import importlib.util
import ast
import re
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class Skill:
    name: str
    description: str
    tags: List[str]
    code: str
    accepted_kwargs: List[str] = field(default_factory=list)
    path: Optional[Path] = None
    preconditions: Optional[Dict[str, Any]] = None
    effects: Optional[Dict[str, Any]] = None


@dataclass
class RetrievedSkill:
    skill_name: str
    description: str
    code: str
    score: float = 0.0
    accepted_kwargs: Optional[List[str]] = None
    preconditions: Optional[Dict[str, Any]] = None
    effects: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.skill_name,
            "description": self.description,
            "code": self.code,
            "score": self.score,
            "accepted_kwargs": self.accepted_kwargs,
            "preconditions": self.preconditions,
            "effects": self.effects,
        }


class SkillLibrary:
    def __init__(
        self, 
        root: str = "skills",
        use_vector_db: bool = True,
        embedding_model: str = "all-MiniLM-L6-v2",
        collection_name: str = "skills",
    ):
        """
        Initialize the skill library.
        
        Args:
            root: Root directory for skill files
            use_vector_db: Use vector DB for retrieval
            embedding_model: Sentence transformer model for embeddings
            collection_name: Name of the vector DB collection
        """
        self.root = Path(root)
        self.seed_dir = self.root / "seed"
        self.gen_dir = self.root / "generated"
        self.gen_dir.mkdir(parents=True, exist_ok=True)
        self.seed_dir.mkdir(parents=True, exist_ok=True)
        
        self._skills: Dict[str, Skill] = {}
        self._use_vector_db = use_vector_db
        self._collection_name = collection_name
        
        # Vector DB components 
        self._embedder = None
        self._vector_db = None
        self._embedding_model = embedding_model
        self._vector_db_initialized = False

    def _init_vector_db(self) -> bool:
        if self._vector_db_initialized:
            return self._vector_db is not None
            
        self._vector_db_initialized = True
        
        if not self._use_vector_db:
            return False
            
        try:
            from vectordb import SkillEmbedder, VectorDBProviderFactory
            
            # Initialize embedder
            self._embedder = SkillEmbedder(self._embedding_model)
            
            # Initialize vector DB
            factory = VectorDBProviderFactory(
                base_dir=str(self.root / "vector_db")
            )
            self._vector_db = factory.create(
                provider="QDRANT",
                db_name="skill_vectors",
                distance_method="cosine"
            )
            self._vector_db.connect()
            
            # Create collection 
            self._vector_db.create_collection(
                collection_name=self._collection_name,
                embedding_size=self._embedder.dim,
                do_reset=False
            )
            
            logger.info("Vector DB initialized successfully")
            return True
            
        except ImportError as e:
            logger.warning(f"Vector DB not available: {e}. Using keyword matching.")
            self._use_vector_db = False
            return False
        except Exception as e:
            logger.warning(f"Vector DB initialization failed: {e}. Using keyword matching.")
            self._use_vector_db = False
            return False

    def load_all(self) -> None:
        self._skills.clear()
        
        for folder in [self.seed_dir, self.gen_dir]:
            if not folder.exists():
                continue
            for py_file in folder.glob("*.py"):
                skill = self._load_skill(py_file)
                if skill:
                    self._skills[skill.name] = skill
                    # Index in vec DB
                    self._index_skill(skill)
        
        logger.info(f"Loaded {len(self._skills)} skills")

    def _index_skill(self, skill: Skill) -> None:
        """Add skill to vec DB index."""
        if not self._init_vector_db():
            return
            
        try:
            embedding = self._embedder.embed_skill(
                name=skill.name,
                description=skill.description,
                tags=skill.tags
            )
            self._vector_db.insert_skill(
                collection_name=self._collection_name,
                embedding=embedding,
                skill_name=skill.name,
                description=skill.description,
                code=skill.code,
                preconditions=skill.preconditions,
                effects=skill.effects,
            )
        except Exception as e:
            logger.warning(f"Failed to index skill '{skill.name}': {e}")

    def list(self) -> List[Dict[str, Any]]:
        return [
            {
                "name": s.name, 
                "description": s.description, 
                "tags": s.tags, 
                "accepted_kwargs": s.accepted_kwargs,
                "path": str(s.path) if s.path else None
            }
            for s in sorted(self._skills.values(), key=lambda x: x.name)
        ]

    def get(self, name: str) -> Optional[Skill]:
        return self._skills.get(name)

    def retrieve(self, query: str, k: int = 8) -> List[Dict[str, Any]]:
        """
        Retrieve relevant skills for a query.Uses vector similarity 
        
        Args:
            query: Task description or search query
            k: Maximum number of results
        Returns:
            List of skill dictionaries sorted by relevance
        """
        if self._init_vector_db() and self._embedder and self._vector_db:
            try:
                embedding = self._embedder.embed(query)
                results = self._vector_db.search_skills(
                    collection_name=self._collection_name,
                    embedding=embedding,
                    limit=k
                )
                if results:
                    enriched = []
                    for result in results:
                        skill_dict = result.to_dict()
                        loaded_skill = self._skills.get(skill_dict.get("name", ""))
                        if loaded_skill:
                            skill_dict["accepted_kwargs"] = loaded_skill.accepted_kwargs
                        enriched.append(skill_dict)
                    return enriched
            except Exception as e:
                logger.warning(f"Vector search failed: {e}. Falling back to keyword.")
        
        return self._keyword_retrieve(query, k)

    def _keyword_retrieve(self, query: str, k: int) -> List[Dict[str, Any]]:
        q = query.lower()
        scored = []
        
        for skill in self._skills.values():
            text = (skill.name + " " + skill.description + " " + " ".join(skill.tags)).lower()
            score = sum(1 for token in q.split() if token in text)
            if score > 0:
                scored.append((score, skill))
        
        scored.sort(key=lambda x: x[0], reverse=True)
        return [
            {
                "name": s.name,
                "description": s.description,
                "tags": s.tags,
                "accepted_kwargs": s.accepted_kwargs,
            }
            for _, s in scored[:k]
        ]

    def save_generated(self, skill_name: str, code: str) -> Path:
        """
        Save a generated skill.
        
        Args:
            skill_name: Name for the skill
            code: Python code implementing the skill
            
        Returns:
            Path where skill saved
        """
        safe_name = "".join(
            ch if ch.isalnum() or ch in "_-" else "_" 
            for ch in skill_name
        )[:80]
        
        path = self.gen_dir / f"{safe_name}.py"
        path.write_text(code, encoding="utf-8")
        
        skill = self._load_skill(path)
        if skill:
            self._skills[skill.name] = skill
            self._index_skill(skill)
        
        return path

    def _load_skill(self, path: Path) -> Optional[Skill]:
        """Load a skill from a Python file."""
        try:
            code = path.read_text(encoding="utf-8")
            
            # Execute module to get metadata
            spec = importlib.util.spec_from_file_location(path.stem, str(path))
            if spec is None or spec.loader is None:
                return None
            
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            
            # Extract skill metadata
            meta = getattr(mod, "SKILL_METADATA", None)
            run_fn = getattr(mod, "run", None)
            
            if not isinstance(meta, dict) or not callable(run_fn):
                logger.warning(f"Invalid skill format: {path}")
                return None

            accepted_kwargs = self._extract_accepted_kwargs(code, meta)
            
            raw_name = str(meta.get("name") or path.stem)
            # Normalize -> snake_case for consistent 
            norm_name = re.sub(r'(?<=[a-z0-9])(?=[A-Z])', '_', raw_name)
            norm_name = norm_name.lower().replace(' ', '_').replace('-', '_')

            return Skill(
                name=norm_name,
                description=str(meta.get("description") or ""),
                tags=list(meta.get("tags") or []),
                code=code,
                accepted_kwargs=accepted_kwargs,
                path=path,
                preconditions=meta.get("preconditions"),
                effects=meta.get("effects"),
            )
            
        except Exception as e:
            logger.warning(f"Failed to load skill from {path}: {e}")
            return None

    def close(self) -> None:
        """Clean up resources."""
        if self._vector_db:
            try:
                self._vector_db.disconnect()
            except Exception:
                pass

    def _extract_accepted_kwargs(self, code: str, meta: Dict[str, Any]) -> List[str]:
        """Extract accepted kwargs from metadata and code usage."""
        accepted: set[str] = set()

        # Optional explicit metadata contract.
        meta_kwargs = meta.get("accepted_kwargs")
        if isinstance(meta_kwargs, list):
            accepted.update(str(k) for k in meta_kwargs if isinstance(k, str))

        # Generic params dictionary support.
        params = meta.get("params")
        if isinstance(params, dict):
            accepted.update(str(k) for k in params.keys())

        # Infer kwargs from usage patterns in the code.
        try:
            tree = ast.parse(code)
        except SyntaxError:
            return sorted(accepted)

        for node in ast.walk(tree):
            # kwargs.get("name", default)
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "get"
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "kwargs"
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and isinstance(node.args[0].value, str)
            ):
                accepted.add(node.args[0].value)

            # kwargs["name"]
            if (
                isinstance(node, ast.Subscript)
                and isinstance(node.value, ast.Name)
                and node.value.id == "kwargs"
            ):
                slice_node = node.slice
                if isinstance(slice_node, ast.Constant) and isinstance(slice_node.value, str):
                    accepted.add(slice_node.value)

        return sorted(accepted)
