"""
Isaac Sim Server
"""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path

import sys
from pathlib import Path

# Add path
sys.path.insert(0, str(Path(__file__).parent.parent))

from server.robot_controller import RobotController
from server.api_server import APIServer
from common.config import (
    ROBOT_PRIM_PATH, 
    ROBOT_USD_PATH, 
    SIMULATION_DT, 
    SERVER_PORT
)
from common.logger import get_server_logger, log_run_info


class RobotSimulation:
    """
    Main simulation class for Robot in Isaac Sim
    """
    
    def __init__(self):
        log_run_info()
        self.logger = get_server_logger()
        self.world = None
        self.robot = None
        self.api_server = None
        
        self.logger.info("="*70)
        self.logger.info("ROBOT ISAAC SIM SERVER")
        self.logger.info("="*70)
    
    def setup_scene(self) -> bool:
        """
        Setup Isaac Sim scene with Robot
        
        Returns:
            True if successful, False otherwise
        """
        self.logger.info("[1/4] Setting up Isaac Sim scene.")
        
        try:
            # Create world
            self.world = World(stage_units_in_meters=1.0)
            self.world.scene.add_default_ground_plane()
            self.logger.info("Ground plane added")
            
            # Get Robot USD
            assets_root = get_assets_root_path()
            if not assets_root:
                self.logger.warning("No assets root found, using default.")
                assets_root = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0"
            
            ROBOT_usd = f"{assets_root}/{ROBOT_USD_PATH}"
            self.logger.info(f"Loading Robot from: {ROBOT_usd}")
            
            # Add Robot
            add_reference_to_stage(usd_path=ROBOT_usd, prim_path=ROBOT_PRIM_PATH)
            self.logger.info("Robot USD loaded")
            
            # Create robot controller
            self.robot = RobotController(ROBOT_PRIM_PATH)
            
            # Reset world
            self.logger.info("Resetting world")
            self.world.reset()
            self.logger.info("World reset complete")
            
            # Init controller
            if self.robot.initialize():
                self.logger.info(f"Robot controller initialized")
            else:
                self.logger.error("Failed to initialize robot controller")
                return False
            
            self.logger.info("Scene setup complete!")
            return True
            
        except Exception as e:
            self.logger.error(f"Scene setup failed: {e}", exc_info=True)
            return False
    
    def start_api_server(self) -> bool:
        """
        Start the API server
        
        Returns:
            True if successful, False otherwise
        """
        self.logger.info("[2/4] Starting API server.")
        
        try:
            self.api_server = APIServer(self.robot, port=SERVER_PORT)
            self.api_server.start()
            self.logger.info("API server started")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start API server: {e}", exc_info=True)
            return False
    
    def run(self):
        """Main simulation loop"""
        self.logger.info("[3/4] Initializing.")
        
        # Setup scene
        if not self.setup_scene():
            self.logger.error("Failed to setup scene. Exiting.")
            return
        
        # Start API server
        if not self.start_api_server():
            self.logger.error("Failed to start API server. Exiting.")
            return
        
        self.logger.info("[4/4] Simulation running.")
        self.logger.info("="*70)
        self.logger.info("READY FOR COMMANDS")
        self.logger.info("="*70)
        self.logger.info(f"Server listening on port {SERVER_PORT}")
        self.logger.info("Run client: python -m client.manual_control")
        self.logger.info("")
        
        # Main simulation loop
        try:
            step_count = 0
            while simulation_app.is_running():
                # Update robot movement
                if self.api_server:
                    self.api_server.update(SIMULATION_DT)
                
                # Step simulation
                self.world.step(render=True)
                
                step_count += 1
                if step_count % 600 == 0:  # Log every 10 sec at 60 fps
                    self.logger.debug(f"Simulation running -> (step {step_count})")
                
        except KeyboardInterrupt:
            self.logger.info("Shutdown requested by user")
        except Exception as e:
            self.logger.error(f"Simulation error: {e}", exc_info=True)
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown"""
        self.logger.info("Shutting down.")
        
        if self.api_server:
            self.api_server.stop()
        
        simulation_app.close()
        self.logger.info("Server stopped. Goodbye!")


if __name__ == '__main__':
    sim = RobotSimulation()
    sim.run()
