from vectordb.providers.qdrant import QdrantDBProvider

def main():
    db = QdrantDBProvider('skills/vector_db/skill_vectors', 'cosine')
    db.connect()
    
    print("="*60)
    print("SKILLS IN VECTOR DATABASE")
    print("="*60)
    
    skills = db.get_all_skills('skills', limit=100)
    print(f"\nTotal skills: {len(skills)}\n")
    
    for i, s in enumerate(skills, 1):
        print(f"{i}. {s.skill_name}")
        print(f"   Description: {s.description}")
        print(f"   Code preview: {s.code[:100]}...")
        print()
    
    db.disconnect()

if __name__ == "__main__":
    main()