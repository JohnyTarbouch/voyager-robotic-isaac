"""
Isaac Sim Server Entry Point
"""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

import sys
from pathlib import Path

# Add path
sys.path.insert(0, str(Path(__file__).parent.parent))

from server.robot_simulation import RobotSimulation
from common.logger import get_server_logger


def main():
    """Main entry point"""
    logger = get_server_logger()
    sim = RobotSimulation()
    
    try:
        # Setup scene
        logger.info("[3/4] Initializing.")
        if not sim.setup_scene():
            logger.error("Failed to setup scene. Exiting.")
            return
        
        # Start API server
        if not sim.start_api_server():
            logger.error("Failed to start API server. Exiting.")
            return
        
        # Run simulation loop
        sim.run_simulation_loop()
        
    except KeyboardInterrupt:
        logger.info("Shutdown requested by user")
    except Exception as e:
        logger.error(f"Simulation error: {e}", exc_info=True)
    finally:
        sim.shutdown()
        simulation_app.close()


if __name__ == '__main__':
    main()
