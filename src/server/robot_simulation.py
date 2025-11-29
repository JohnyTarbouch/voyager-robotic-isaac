"""
Robot Simulation Class
"""

from server.robot_controller import RobotController
from server.api_server import APIServer
from manual_cmd.env_setup import EnvironmentSetup
from common.config import SIMULATION_DT, SERVER_PORT
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
            env = EnvironmentSetup()
            self.world, self.robot = env.setup()
            
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
    
    def run_simulation_loop(self):
        """Main simulation loop"""
        self.logger.info("[4/4] Simulation running.")
        self.logger.info("="*70)
        self.logger.info("READY FOR COMMANDS")
        self.logger.info("="*70)
        self.logger.info(f"Server listening on port {SERVER_PORT}")
        self.logger.info("Run client: python -m manual_cmd.manual_control")
        self.logger.info("")
        
        step_count = 0
        while True:
            # Update robot movement
            if self.api_server:
                self.api_server.update(SIMULATION_DT)
            
            # Step simulation
            self.world.step(render=True)
            
            step_count += 1
            if step_count % 600 == 0:  # Log every 10 sec at 60 fps
                self.logger.debug(f"Simulation running -> (step {step_count})")
    
    def shutdown(self):
        self.logger.info("Shutting down.")
        
        if self.api_server:
            self.api_server.stop()
        
        self.logger.info("Simulation stopped.")
