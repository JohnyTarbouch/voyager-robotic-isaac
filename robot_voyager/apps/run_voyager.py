"""
Run Voyager-Style Autonomous Agent

Usage:
    python -m apps.run_voyager --max-tasks 20

Like the original Voyager paper:
- Agent proposes ANY task it wants
- LLM verifies task completion
- Continuous learning with 4 cubes (for manipulation)
"""

import argparse
import logging
import sys

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)s | %(name)s | %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description="Run Voyager-style autonomous agent")
    # Maax tasks to attempt, so LLM stops proposing new ones
    parser.add_argument("--max-tasks", type=int, default=20, help="Max tasks to attempt") 
    parser.add_argument("--headless", action="store_true", help="Run without visualization")
    args = parser.parse_args()
    
    print("="*60)
    print("VOYAGER-STYLE AUTONOMOUS LEARNING")
    print("="*60)
    print("1.Open ended curriculum (LLM proposes tasks)")
    print("2. LLM critic verification")
    print("3. 4 cubes for manipulation")
    print("="*60)
    print()
    
    # Import Isaac Sim
    try:
        from enviroment.isaac_franka import IsaacFrankaRobotAPI
        robot = IsaacFrankaRobotAPI(headless=args.headless)
        logger.info("Isaac Sim initialised")
    except ImportError as e:
        logger.error(f"Failed to import Isaac Sim: {e}")
        logger.error("Make sure u runing Isaac Sim.")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Failed to initialize Isaac Sim: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    # Import and run Voyager
    try:
        from agent.voyager_runner import VoyagerRunner
        from cfg.settings import Config
        
        config = Config()
        
        # Try to use rich console (Isaac sim franka)
        try:
            from rich.console import Console
            console = Console()
        except ImportError:
            console = None
        
        runner = VoyagerRunner(robot, config, args.max_tasks, console)
        runner.run()
        
        print("\nVoyager session complete!")
        print(f"Tasks completed: {runner.tasks_completed}")
        print(f"Tasks failed: {runner.tasks_failed}")
        print(f"Skills learned: {runner.skills_learned}")

    except Exception as e:
        logger.error(f"Voyager agent failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()