"""
Environment and Robot Setup
"""
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

from server.robot_controller import RobotController
from common.config import ROBOT_PRIM_PATH, ROBOT_USD_PATH
from common.logger import get_server_logger


class EnvironmentSetup:
    """Setup environment with robot and obstacles"""
    
    def __init__(self):
        self.logger = get_server_logger()
        self.world = None
        self.robot = None
    
    def setup(self):
        """Setup world, robot, and obstacles"""
        self.logger.info("Setting up environment...")
        
        # Create world
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        
        # Add robot
        assets_root = get_assets_root_path()
        if not assets_root:
            assets_root = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0"
        
        robot_usd = f"{assets_root}/{ROBOT_USD_PATH}"
        add_reference_to_stage(usd_path=robot_usd, prim_path=ROBOT_PRIM_PATH)
        self.logger.info("Robot loaded")
        
        # Add obstacle 
        self.obstacle = DynamicCuboid(
            prim_path="/World/Obstacle",
            position=np.array([2.0, 0.0, 0.5]),
            size=0.5,
            color=np.array([1.0, 0.0, 0.0])
        )
        self.logger.info("Obstacle added")
        
        # Reset world
        self.world.reset()
        
        # Initialize robot controller
        self.robot = RobotController(ROBOT_PRIM_PATH)
        self.robot.initialize()
        self.logger.info("Environment setup complete")
        
        return self.world, self.robot