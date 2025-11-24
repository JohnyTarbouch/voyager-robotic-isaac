import logging
import sys
from pathlib import Path
from typing import Optional
from .config import (
    LOG_LEVEL, LOG_FORMAT, LOG_DATE_FORMAT,
    SERVER_LOG_FILE, CLIENT_LOG_FILE, COMMANDS_LOG_FILE, SKILLS_LOG_FILE
)


def setup_logger(
    name: str,
    log_file: Optional[Path] = None,
    level: str = LOG_LEVEL,
    console: bool = True
) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(getattr(logging, level.upper()))
    
    logger.handlers.clear()
    
    formatter = logging.Formatter(LOG_FORMAT, datefmt=LOG_DATE_FORMAT)
    
    if log_file:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        file_handler = logging.FileHandler(log_file, mode='a')
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    
    if console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(getattr(logging, level.upper()))
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
    
    return logger


def get_server_logger() -> logging.Logger:
    return setup_logger('server', SERVER_LOG_FILE)


def get_client_logger() -> logging.Logger:
    return setup_logger('client', CLIENT_LOG_FILE)


def get_commands_logger() -> logging.Logger:
    logger = setup_logger('commands', COMMANDS_LOG_FILE, console=False)
    return logger


def get_skills_logger() -> logging.Logger:
    return setup_logger('skills', SKILLS_LOG_FILE, console=False)


class CommandLogger:    
    def __init__(self):
        self.logger = get_commands_logger()
    
    def log_command(self, action: str, params: dict, success: bool, 
                    duration: float = 0.0, error: str = None):
        log_data = {
            'action': action,
            'params': params,
            'success': success,
            'duration_ms': round(duration * 1000, 2)
        }
        
        if error:
            log_data['error'] = error
        
        if success:
            self.logger.info(f"Command executed: {log_data}")
        else:
            self.logger.error(f"Command failed: {log_data}")
    
    def log_skill_execution(self, skill_name: str, code: str, 
                            success: bool, error: str = None):
        log_data = {
            'skill': skill_name,
            'code_preview': code[:100] + '...' if len(code) > 100 else code,
            'success': success
        }
        
        if error:
            log_data['error'] = error
        
        if success:
            self.logger.info(f"Skill executed: {log_data}")
        else:
            self.logger.error(f"Skill failed: {log_data}")
