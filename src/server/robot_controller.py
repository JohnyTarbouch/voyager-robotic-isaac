import numpy as np
from scipy.spatial.transform import Rotation
from omni.isaac.core.prims import XFormPrim
from typing import Tuple


class RobotController:
    """
    Simple controller using direct rigid body manipulation
    """
    
    def __init__(self, prim_path: str):
        self.prim_path = prim_path
        self.xform = None
        
    def initialize(self) -> bool:
        try:
            self.xform = XFormPrim(self.prim_path)
            return True
        except Exception as e:
            return False
        
    def move_by(self, forward: float = 0.0, sideways: float = 0.0, 
                rotation: float = 0.0, dt: float = 0.01) -> bool:
        """
        Move robot by specified amounts
        
        Args:
            forward: Forward movement
            sideways: Sideways movement
            rotation: Rotation amount
            dt: Delta time 
        
        Returns:
            True if successful, False otherwise
        """
        if not self.xform:
            return False
            
        try:
            # Get current pos
            position, orientation = self.xform.get_world_pose()
            
            quat_scipy = [orientation[1], orientation[2], orientation[3], orientation[0]]
            rot = Rotation.from_quat(quat_scipy)
            
            # Get forward and right directions
            forward_dir = rot.apply([1, 0, 0])
            right_dir = rot.apply([0, -1, 0])
            
            # Calculate new position
            new_position = position + forward_dir * forward * dt + right_dir * sideways * dt
            
            # Calculate new orientation
            if abs(rotation) > 0.001:
                # Rotate around Z ax
                delta_rot = Rotation.from_euler('z', rotation * dt, degrees=False)
                new_rot = delta_rot * rot
                new_quat_scipy = new_rot.as_quat()
                new_orientation = np.array([
                    new_quat_scipy[3], new_quat_scipy[0],
                    new_quat_scipy[1], new_quat_scipy[2]
                ])
            else:
                new_orientation = orientation
            # Set new pose
            self.xform.set_world_pose(position=new_position, orientation=new_orientation)
            return True
            
        except Exception as e:
            return False
    
    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.xform:
            return self.xform.get_world_pose()
        return np.array([0, 0, 0]), np.array([1, 0, 0, 0])
    
    def get_position(self) -> dict:
        position, _ = self.get_world_pose()
        return {
            'x': float(position[0]),
            'y': float(position[1]),
            'z': float(position[2])
        }
    
    def get_orientation(self) -> dict:
        _, orientation = self.get_world_pose()
        return {
            'w': float(orientation[0]),
            'x': float(orientation[1]),
            'y': float(orientation[2]),
            'z': float(orientation[3])
        }
