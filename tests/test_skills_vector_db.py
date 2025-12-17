from src.vectordb.vectordb_providor import VectorDBProviderFactory
from src.common import config

def list_all_skills():
    # Create database provider (Qdrant)
    factory = VectorDBProviderFactory(config)
    provider = factory.create(config.VECTOR_DB_PROVIDER)

    # Connect to local Qdrant DB
    provider.connect()

    collection_name = config.SKILLS_COLLECTION_NAME

    print(f"Listing all skills in collection: '{collection_name}'")

    # Check that the collection exists
    if not provider.is_collection_existed(collection_name):
        print("Skills collection does not exist.")
        return

    # --- Qdrant scroll to retrieve all points ---
    all_skills = []
    next_offset = None

    while True:
        result = provider.client.scroll(
            collection_name=collection_name,
            limit=200,  # adjust as needed
            offset=next_offset,
            with_payload=True,
            with_vectors=False
        )

        points, next_offset = result
        all_skills.extend(points)

        if next_offset is None:
            break

    # Display results
    print(f"\nFound {len(all_skills)} skills:\n")

    for p in all_skills:
        payload = p.payload
        print("--------------------------------------------------")
        print(f"Skill Name : {payload.get('skill_name')}")
        print(f"Description: {payload.get('description')}")
        print(f"Code:\n{payload.get('code')}")
        print(f"Preconditions: {payload.get('preconditions')}")
        print(f"Effects:       {payload.get('effects')}")
        print("--------------------------------------------------\n")

    provider.disconnect()


if __name__ == "__main__":
    list_all_skills()
