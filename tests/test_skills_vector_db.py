from src.vectordb.vectordb_providor import VectorDBProviderFactory
from src.vectordb.vectoredb_enums import VectorDBEnums, DistanceMethodEnums
from src.vectordb.skill_embedding import SkillEmbedder
from src.skill_library.skill_store import SkillStore
from src.common.config import SKILL_EMBEDDING_MODEL


class TestConfig:
    VECTOR_DB_PATH = "isaac_skills"
    VECTOR_DB_DISTANCE_METHOD = DistanceMethodEnums.COSINE.value


embedder = SkillEmbedder(SKILL_EMBEDDING_MODEL)
EMBEDDING_DIM = embedder.dim

factory = VectorDBProviderFactory(TestConfig())
db = factory.create(VectorDBEnums.QDRANT.value)
db.connect()

COLLECTION_NAME = "skills"

db.create_collection(
    collection_name=COLLECTION_NAME,
    embedding_size=EMBEDDING_DIM,
    do_reset=True,
)

db.insert_skill(
    collection_name=COLLECTION_NAME,
    embedding=embedder.embed("Pick up a cube from the table."),
    skill_name="pick_cube",
    description="Pick up a cube from the table using the robot gripper.",
    code="""
def execute(env):
    env.arm.move_to("cube")
    env.arm.close_gripper()
""",
    preconditions={"cube_visible": True},
    effects={"holding_cube": True},
)

db.insert_skill(
    collection_name=COLLECTION_NAME,
    embedding=embedder.embed("Rotate robot base by 90 degrees."),
    skill_name="rotate_base",
    description="Rotate the robot base by 90 degrees clockwise.",
    code="""
def execute(env):
    env.base.rotate(90)
""",
)

store = SkillStore(db=db)

results = store.retrieve("grab the cube")

print("\n=== Retrieved Skills ===")
for skill in results:
    print(f"\nScore: {skill.score:.4f}")
    print(f"Skill: {skill.skill_name}")
    print(f"Description: {skill.description}")

db.disconnect()
