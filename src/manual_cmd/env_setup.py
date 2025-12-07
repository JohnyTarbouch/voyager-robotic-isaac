"""
Environment Setup for Isaac Sim with Articulation Control
Updated to use physics-based robot control
"""

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from server.robot_controller import RobotController
from common.config import ROBOT_PRIM_PATH, ROBOT_USD_PATH
import carb


class EnvironmentSetup:
    """Setup Isaac Sim environment with physics-based robot"""
    
    def __init__(self):
        self.world = None
        self.robot = None
    
    def setup(self):
        """
        Setup the Isaac Sim world and robot with articulation control
        
        Returns:
            tuple: (world, robot_controller)
        """
        print("="*70)
        print("ENVIRONMENT SETUP - Starting")
        print("="*70)
        
        # Step 1: Create world
        print("[1/5] Creating World...")
        self.world = World(stage_units_in_meters=1.0)
        print("      World created")
        
        # Step 2: Add ground plane
        print("[2/5] Adding ground plane...")
        self.world.scene.add_default_ground_plane()
        print("      Ground plane added")
        
        # Step 3: Get robot asset path
        print("[3/5] Getting robot asset path...")
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            error_msg = "Could not find nucleus server with /Isaac folder"
            carb.log_error(error_msg)
            raise RuntimeError(error_msg)
        
        # Build full asset path
        asset_path = assets_root_path + "/" + ROBOT_USD_PATH
        print(f"      Asset path: {asset_path}")
        
        # Step 4: Initialize robot controller
        print("[4/5] Initializing robot controller...")
        self.robot = RobotController(
            prim_path=ROBOT_PRIM_PATH,
            usd_path=asset_path
        )
        
        # Add robot to the world scene
        if not self.robot.initialize(self.world):
            raise RuntimeError("Failed to initialize robot controller")
        print("      Robot controller initialized")
        
        # Step 5: CRITICAL - Reset world to initialize physics
        print("[5/5] Resetting world (initializing physics)...")
        self.world.reset()
        print("      Physics initialized")
        
        print("="*70)
        print("ENVIRONMENT SETUP - Complete")
        print("="*70)
        
        return self.world, self.robot