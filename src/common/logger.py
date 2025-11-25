import logging
import sys
from pathlib import Path
from .config import (
    LOG_LEVEL, LOG_FORMAT, LOG_DATE_FORMAT,
    CONSOLIDATED_LOG_FILE, RUN_LOG_DIR
)


def setup_logger(
    name: str,
    log_file: Path = CONSOLIDATED_LOG_FILE,
    level: str = LOG_LEVEL,
    console: bool = True
) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(getattr(logging, level.upper()))

    logger.handlers.clear()

    formatter = logging.Formatter(LOG_FORMAT, datefmt=LOG_DATE_FORMAT)

    file_handler = logging.FileHandler(log_file, mode='a')
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    if console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(getattr(logging, level.upper()))
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    logger.propagate = False
    return logger


def get_server_logger() -> logging.Logger:
    return setup_logger('server', console=True)


def get_manualcmd_logger() -> logging.Logger:
    return setup_logger('manualcmd', console=True)


def get_commands_logger() -> logging.Logger:
    return setup_logger('commands', console=False)


def get_skills_logger() -> logging.Logger:
    return setup_logger('skills', console=False)


class CommandLogger:
    def __init__(self):
        self.logger = get_commands_logger()
        self.logger.info(f"CommandLogger initialized - logging to {CONSOLIDATED_LOG_FILE}")

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


def log_run_info():
    """
    Updated so it uses the SAME logger instead of creating a second file handler.
    """
    logger = get_server_logger()
    logger.info("=" * 70)
    logger.info("NEW RUN STARTED")
    logger.info(f"Log Directory: {RUN_LOG_DIR}")
    logger.info(f"Log File: {CONSOLIDATED_LOG_FILE}")
    logger.info("=" * 70)
