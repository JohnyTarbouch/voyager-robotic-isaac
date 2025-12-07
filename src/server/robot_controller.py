import numpy as np
from scipy.spatial.transform import Rotation
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.types import ArticulationAction
from typing import Tuple, Optional


class RobotController:
    """
    Physics-based controller using WheeledRobot class
    Migrated from XFormPrim to articulation control
    """
    
    def __init__(self, prim_path: str, usd_path: str = None):
        """
        Initialize robot controller
        
        Args:
            prim_path: Path to robot prim in stage
            usd_path: Path to robot USD file (required for WheeledRobot)
        """
        self.prim_path = prim_path
        self.usd_path = usd_path
        self.robot = None
        
        # Jetbot wheel parameters
        self.wheel_radius = 0.03  # meters (3 cm)
        self.wheel_base = 0.1125  # meters (11.25 cm between wheels)
        
        print(f"[RobotController] Initialized with path: {prim_path}")
    
    def initialize(self, world) -> bool:
        """
        Initialize the wheeled robot and add to world
        
        Args:
            world: Isaac Sim World object
            
        Returns:
            True if successful
        """
        try:
            print(f"[RobotController] Creating WheeledRobot at {self.prim_path}")
            
            # Create WheeledRobot instance
            self.robot = world.scene.add(
                WheeledRobot(
                    prim_path=self.prim_path,
                    name="jetbot",
                    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                    create_robot=True,
                    usd_path=self.usd_path,
                )
            )
            
            print("[RobotController] WheeledRobot created successfully")
            return True
            
        except Exception as e:
            print(f"[RobotController] ERROR: Failed to initialize robot: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def move_by(self, forward: float = 0.0, sideways: float = 0.0, 
                rotation: float = 0.0, dt: float = 0.01) -> bool:
        """
        Move robot using differential drive kinematics
        
        IMPORTANT: This now applies VELOCITIES continuously, not incremental movement
        The robot will continue moving until you call this again with different values
        
        Args:
            forward: Forward velocity in m/s (positive = forward)
            sideways: Sideways velocity in m/s (IGNORED - differential drive)
            rotation: Angular velocity in rad/s (positive = counter-clockwise)
            dt: Delta time (kept for compatibility but not used in velocity control)
        
        Returns:
            True if successful
        """
        if not self.robot:
            print("[RobotController] ERROR: Robot not initialized")
            return False
        
        try:
            # Convert linear velocity (m/s) and angular velocity (rad/s) 
            # to wheel angular velocities (rad/s)
            # 
            # Differential drive kinematics:
            # v_left = (2*v + ω*L) / (2*R)
            # v_right = (2*v - ω*L) / (2*R)
            
            v = forward  # linear velocity (m/s)
            omega = rotation  # angular velocity (rad/s)
            
            # Calculate individual wheel velocities (rad/s)
            v_left = (2*v + omega*self.wheel_base) / (2*self.wheel_radius)
            v_right = (2*v - omega*self.wheel_base) / (2*self.wheel_radius)
            
            # Apply velocities to wheels
            self.robot.apply_wheel_actions(
                ArticulationAction(
                    joint_positions=None,
                    joint_efforts=None,
                    joint_velocities=np.array([v_left, v_right])
                )
            )
            
            return True
            
        except Exception as e:
            print(f"[RobotController] ERROR: Failed to move robot: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def apply_wheel_velocities(self, left_velocity: float, right_velocity: float) -> bool:
        """
        Directly set wheel velocities (for advanced control)
        
        Args:
            left_velocity: Left wheel angular velocity in rad/s
            right_velocity: Right wheel angular velocity in rad/s
        """
        if not self.robot:
            return False
        
        try:
            self.robot.apply_wheel_actions(
                ArticulationAction(
                    joint_positions=None,
                    joint_efforts=None,
                    joint_velocities=np.array([left_velocity, right_velocity])
                )
            )
            return True
        except Exception as e:
            print(f"[RobotController] ERROR: {e}")
            return False
    
    def stop(self) -> bool:
        """Stop the robot by setting wheel velocities to zero"""
        return self.apply_wheel_velocities(0.0, 0.0)
    
    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get robot's world position and orientation"""
        if self.robot:
            return self.robot.get_world_pose()
        return np.array([0, 0, 0]), np.array([1, 0, 0, 0])
    
    def get_position(self) -> dict:
        """Get robot position as dictionary"""
        position, _ = self.get_world_pose()
        return {
            'x': float(position[0]),
            'y': float(position[1]),
            'z': float(position[2])
        }
    
    def get_orientation(self) -> dict:
        """Get robot orientation as quaternion dictionary"""
        _, orientation = self.get_world_pose()
        return {
            'w': float(orientation[0]),
            'x': float(orientation[1]),
            'y': float(orientation[2]),
            'z': float(orientation[3])
        }
    
    def get_linear_velocity(self) -> Optional[np.ndarray]:
        """Get robot's linear velocity"""
        if self.robot:
            return self.robot.get_linear_velocity()
        return None
    
    def get_angular_velocity(self) -> Optional[np.ndarray]:
        """Get robot's angular velocity"""
        if self.robot:
            return self.robot.get_angular_velocity()
        return None