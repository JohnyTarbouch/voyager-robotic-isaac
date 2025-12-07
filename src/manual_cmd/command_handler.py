"""
Command Handler - Updated for articulation-based control
Key change: Movement is now velocity-based, not position-based
"""

from typing import Dict, Any
import math
import time


class CommandHandler:
    """
    Handle robot commands with velocity-based movement
    """
    
    def __init__(self, robot_controller, logger):
        self.robot = robot_controller
        self.logger = logger
        
        # Movement state for timed commands
        self.current_velocity = {'forward': 0.0, 'rotation': 0.0}
        self.movement_start_time = None
        self.movement_duration = 0.0
        self.is_moving = False
        
        self.logger.info("CommandHandler initialized (velocity-based control)")
    
    def handle(self, action: str, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handle incoming command
        
        Args:
            action: Command action string
            params: Command parameters
            
        Returns:
            Response dictionary
        """
        handlers = {
            'move_forward': self._handle_move_forward,
            'move_backward': self._handle_move_backward,
            'turn_left': self._handle_turn_left,
            'turn_right': self._handle_turn_right,
            'stop': self._handle_stop,
            'get_position': self._handle_get_position,
            'get_orientation': self._handle_get_orientation,
            'custom_move': self._handle_custom_move,
        }
        
        handler = handlers.get(action)
        if handler:
            return handler(params)
        else:
            return {
                'success': False,
                'error': f'Unknown action: {action}'
            }
    
    def _handle_move_forward(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Move forward for specified distance or time"""
        distance = params.get('distance', 1.0)  # meters
        speed = params.get('speed', 0.5)  # m/s
        
        # Calculate duration needed
        duration = distance / speed
        
        # Start movement
        self.current_velocity = {'forward': speed, 'rotation': 0.0}
        self.movement_start_time = time.time()
        self.movement_duration = duration
        self.is_moving = True
        
        self.robot.move_by(forward=speed, rotation=0.0)
        
        return {
            'success': True,
            'message': f'Moving forward {distance}m at {speed}m/s',
            'duration': duration
        }
    
    def _handle_move_backward(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Move backward for specified distance or time"""
        distance = params.get('distance', 1.0)
        speed = params.get('speed', 0.5)
        
        duration = distance / speed
        
        self.current_velocity = {'forward': -speed, 'rotation': 0.0}
        self.movement_start_time = time.time()
        self.movement_duration = duration
        self.is_moving = True
        
        self.robot.move_by(forward=-speed, rotation=0.0)
        
        return {
            'success': True,
            'message': f'Moving backward {distance}m at {speed}m/s',
            'duration': duration
        }
    
    def _handle_turn_left(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Turn left by specified angle"""
        angle = params.get('angle', 90)  # degrees
        angular_speed = params.get('angular_speed', 1.0)  # rad/s
        
        # Convert angle to radians
        angle_rad = math.radians(angle)
        
        # Calculate duration
        duration = abs(angle_rad / angular_speed)
        
        # Start rotation (positive = counter-clockwise = left)
        self.current_velocity = {'forward': 0.0, 'rotation': angular_speed}
        self.movement_start_time = time.time()
        self.movement_duration = duration
        self.is_moving = True
        
        self.robot.move_by(forward=0.0, rotation=angular_speed)
        
        return {
            'success': True,
            'message': f'Turning left {angle}° at {angular_speed}rad/s',
            'duration': duration
        }
    
    def _handle_turn_right(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Turn right by specified angle"""
        angle = params.get('angle', 90)
        angular_speed = params.get('angular_speed', 1.0)
        
        angle_rad = math.radians(angle)
        duration = abs(angle_rad / angular_speed)
        
        # Negative rotation = clockwise = right
        self.current_velocity = {'forward': 0.0, 'rotation': -angular_speed}
        self.movement_start_time = time.time()
        self.movement_duration = duration
        self.is_moving = True
        
        self.robot.move_by(forward=0.0, rotation=-angular_speed)
        
        return {
            'success': True,
            'message': f'Turning right {angle}° at {angular_speed}rad/s',
            'duration': duration
        }
    
    def _handle_stop(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Stop the robot immediately"""
        self.robot.stop()
        self.is_moving = False
        self.current_velocity = {'forward': 0.0, 'rotation': 0.0}
        
        return {
            'success': True,
            'message': 'Robot stopped'
        }
    
    def _handle_get_position(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Get current robot position"""
        position = self.robot.get_position()
        return {
            'success': True,
            'position': position
        }
    
    def _handle_get_orientation(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Get current robot orientation"""
        orientation = self.robot.get_orientation()
        return {
            'success': True,
            'orientation': orientation
        }
    
    def _handle_custom_move(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Custom movement with direct velocity control
        
        Params:
            forward: forward velocity (m/s)
            rotation: angular velocity (rad/s)
            duration: how long to move (seconds), optional
        """
        forward = params.get('forward', 0.0)
        rotation = params.get('rotation', 0.0)
        duration = params.get('duration', None)
        
        if duration:
            # Timed movement
            self.current_velocity = {'forward': forward, 'rotation': rotation}
            self.movement_start_time = time.time()
            self.movement_duration = duration
            self.is_moving = True
        
        self.robot.move_by(forward=forward, rotation=rotation)
        
        return {
            'success': True,
            'message': f'Custom move: forward={forward}, rotation={rotation}',
            'duration': duration
        }
    
    def update(self, dt: float):
        """
        Update movement state (called every simulation step)
        
        This checks if timed movements should stop
        
        Args:
            dt: Delta time since last update
        """
        if not self.is_moving:
            return
        
        # Check if movement duration has elapsed
        elapsed = time.time() - self.movement_start_time
        
        if elapsed >= self.movement_duration:
            # Stop the robot
            self.robot.stop()
            self.is_moving = False
            self.current_velocity = {'forward': 0.0, 'rotation': 0.0}
            self.logger.debug("Timed movement completed, robot stopped")