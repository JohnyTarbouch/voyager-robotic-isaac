import os
from pathlib import Path
from datetime import datetime

PROJECT_ROOT = Path(__file__).parent.parent.parent
LOGS_DIR = PROJECT_ROOT / "logs"
DATA_DIR = PROJECT_ROOT / "data"

LOGS_DIR.mkdir(exist_ok=True)
DATA_DIR.mkdir(exist_ok=True)

RUN_TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
RUN_LOG_DIR = LOGS_DIR / f"run_{RUN_TIMESTAMP}"
RUN_LOG_DIR.mkdir(exist_ok=True)

SERVER_HOST = os.getenv('JETBOT_SERVER_HOST', 'localhost')
SERVER_PORT = int(os.getenv('JETBOT_SERVER_PORT', '8888'))

ROBOT_PRIM_PATH = "/World/Jetbot"
ROBOT_USD_PATH = "Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"

DEFAULT_LINEAR_SPEED = 0.5  # m/s
DEFAULT_ANGULAR_SPEED = 1.0  # rad/s
DEFAULT_DISTANCE = 1.0  # meters
DEFAULT_ANGLE = 90  # degrees

SIMULATION_FPS = 60
SIMULATION_DT = 1.0 / SIMULATION_FPS

# Database
SKILLS_DB_PATH = DATA_DIR / "skills.db"

# Logging
LOG_LEVEL = os.getenv('LOG_LEVEL', 'INFO')
LOG_FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
LOG_DATE_FORMAT = '%Y-%m-%d %H:%M:%S'

CONSOLIDATED_LOG_FILE = RUN_LOG_DIR / "robot_system.log"

# LLM config
LLM_PROVIDER = os.getenv('LLM_PROVIDER', 'anthropic') 
LLM_MODEL = os.getenv('LLM_MODEL', 'claude-sonnet-4-20250514')
LLM_MAX_RETRIES = 3

# API keys
ANTHROPIC_API_KEY = os.getenv('ANTHROPIC_API_KEY')
OPENAI_API_KEY = os.getenv('OPENAI_API_KEY')
