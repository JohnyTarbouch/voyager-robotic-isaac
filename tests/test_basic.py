"""
Basic tests for the project
"""

import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from common.config import SERVER_PORT, ROBOT_PRIM_PATH
from common.logger import setup_logger


def test_config_imports():
    """Test confi"""
    assert SERVER_PORT == 8888
    assert ROBOT_PRIM_PATH == "/World/Jetbot"


def test_logger_creation():
    """Test logger"""
    logger = setup_logger('test', console=True)
    assert logger is not None
    assert logger.name == 'test'


def test_project_structure():
    """Test project directories exist"""
    project_root = Path(__file__).parent.parent
    
    assert (project_root / 'src').exists()
    assert (project_root / 'src' / 'server').exists()
    assert (project_root / 'src' / 'client').exists()
    assert (project_root / 'src' / 'common').exists()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
