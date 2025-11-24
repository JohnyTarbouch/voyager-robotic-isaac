import sqlite3
from typing import List, Dict, Optional, Any
from datetime import datetime

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from common.config import SKILLS_DB_PATH
from common.logger import get_skills_logger


class SkillLibrary:
    """
    Database to store and manage learned skills
    """
    
    def __init__(self, db_path: Path = SKILLS_DB_PATH):
        """
        Initialize skill library
        
        Args:
            db_path: Path to SQLite database
        """
        self.db_path = db_path
        self.logger = get_skills_logger()
        self._init_database()
        
        self.logger.info(f"Skill library initialized at {db_path}")
    
    def _init_database(self):
        """Initialize SQLite database"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS skills (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT UNIQUE NOT NULL,
                code TEXT NOT NULL,
                description TEXT,
                success_count INTEGER DEFAULT 0,
                failure_count INTEGER DEFAULT 0,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                last_used TIMESTAMP
            )
        ''')
        
        conn.commit()
        conn.close()
        
        self.logger.debug("Database initialized")
    
    def add_skill(self, name: str, code: str, description: str = '') -> bool:
        """
        Add new skill to library
        
        Args:
            name: Skill name
            code: Python code
            description: Skill description
        
        Returns:
            True if added, False if already exists
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        try:
            cursor.execute('''
                INSERT INTO skills (name, code, description)
                VALUES (?, ?, ?)
            ''', (name, code, description))
            conn.commit()
            
            self.logger.info(f"Skill added: {name}")
            return True
            
        except sqlite3.IntegrityError:
            self.logger.warning(f"Skill already exists: {name}")
            return False
            
        finally:
            conn.close()
    
    def get_skill(self, name: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve skill by name
        
        Args:
            name: Skill name
        
        Returns:
            Dictionary with skill data
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT code, description, success_count, failure_count 
            FROM skills WHERE name = ?
        ''', (name,))
        result = cursor.fetchone()
        conn.close()
        
        if result:
            self.logger.debug(f"Retrieved skill: {name}")
            return {
                'code': result[0],
                'description': result[1],
                'success_count': result[2],
                'failure_count': result[3]
            }
        
        self.logger.debug(f"Skill not found: {name}")
        return None
    
    def list_skills(self) -> List[tuple]:
        """
        List all skills
        
        Returns:
            List of tuples (name, description, success_count, failure_count)
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT name, description, success_count, failure_count 
            FROM skills 
            ORDER BY success_count DESC
        ''')
        skills = cursor.fetchall()
        conn.close()
        
        self.logger.debug(f"Listed {len(skills)} skills")
        return skills
    
    def get_all_skills(self) -> List[Dict[str, Any]]:
        """
        Get all skills as list of dictionaries
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT name, code, description, success_count 
            FROM skills 
            WHERE success_count > 0
            ORDER BY success_count DESC
        ''')
        skills = cursor.fetchall()
        conn.close()
        
        return [
            {
                'name': s[0],
                'code': s[1],
                'description': s[2],
                'success_count': s[3]
            } for s in skills
        ]
    
    def record_success(self, name: str):
        """
        Record successful execution of skill
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            UPDATE skills 
            SET success_count = success_count + 1,
                last_used = CURRENT_TIMESTAMP
            WHERE name = ?
        ''', (name,))
        
        conn.commit()
        conn.close()
        
        self.logger.info(f"Skill success recorded: {name}")
    
    def record_failure(self, name: str):
        """
        Record failed execution of skill
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            UPDATE skills 
            SET failure_count = failure_count + 1,
                last_used = CURRENT_TIMESTAMP
            WHERE name = ?
        ''', (name,))
        
        conn.commit()
        conn.close()
        
        self.logger.warning(f"Skill failure recorded: {name}")
    
    def delete_skill(self, name: str) -> bool:
        """
        Delete a skill
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('DELETE FROM skills WHERE name = ?', (name,))
        deleted = cursor.rowcount > 0
        
        conn.commit()
        conn.close()
        
        if deleted:
            self.logger.info(f"Skill deleted: {name}")
        else:
            self.logger.warning(f"Skill not found for deletion: {name}")
        
        return deleted
    
    def get_stats(self) -> Dict[str, Any]:
        """
        Get library statistics
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT 
                COUNT(*) as total_skills,
                SUM(success_count) as total_successes,
                SUM(failure_count) as total_failures
            FROM skills
        ''')
        result = cursor.fetchone()
        conn.close()
        
        return {
            'total_skills': result[0] or 0,
            'total_successes': result[1] or 0,
            'total_failures': result[2] or 0
        }
