from typing import Dict, Any


class CommandHandler:
    """
    Handles robot commands and movement state
    """
    
    def __init__(self, robot_controller, logger):
        """
        Initialize command handler
        
        Args:
            robot_controller: Robot controller instance
            logger: Logger instance
        """
        self.robot = robot_controller
        self.logger = logger
        
        # Movement state
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
    
    def handle(self, action: str, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handle a command action
        
        Args:
            action: Action to perform
            params: Parameters for the action
        
        Returns:
            Response dictionary
        """
        if action == 'move_forward':
            return self._move_forward(params)
            
        elif action == 'move_backward':
            return self._move_backward(params)
            
        elif action == 'turn_left':
            return self._turn_left(params)
            
        elif action == 'turn_right':
            return self._turn_right(params)
            
        elif action == 'stop':
            return self._stop()
            
        elif action == 'get_state':
            return self._get_state()
            
        else:
            self.logger.warning(f"Unknown action: {action}")
            return {'success': False, 'error': f'Unknown action: {action}'}
    
    def update(self, dt: float):
        """
        Update robot movement
        
        Args:
            dt: Delta time since last update
        """
        if abs(self.linear_velocity) > 0.001 or abs(self.angular_velocity) > 0.001:
            self.robot.move_by(
                forward=self.linear_velocity,
                rotation=self.angular_velocity,
                dt=dt
            )
            self.logger.debug(
                f"Robot updated: linear_velocity={self.linear_velocity}, "
                f"angular_velocity={self.angular_velocity}, dt={dt}"
            )
    
    # Command implementations
    
    def _move_forward(self, params: Dict[str, Any]) -> Dict[str, Any]:
        speed = params.get('speed', 0.5)
        distance = params.get('distance', 1.0)
        
        self.linear_velocity = speed
        self.angular_velocity = 0.0
        duration = distance / speed
        
        self.logger.debug(f"Moving forward: {distance} at {speed} (duration: {duration:.2f}s)")
        
        return {
            'success': True,
            'duration': duration,
            'message': f'Moving forward {distance} at {speed}'
        }
    
    def _move_backward(self, params: Dict[str, Any]) -> Dict[str, Any]:
        speed = params.get('speed', 0.5)
        distance = params.get('distance', 1.0)
        
        self.linear_velocity = -speed
        self.angular_velocity = 0.0
        duration = distance / speed
        
        self.logger.debug(f"Moving backward: {distance} at {speed}")
        
        return {
            'success': True,
            'duration': duration,
            'message': f'Moving backward {distance} at {speed}'
        }
    
    def _turn_left(self, params: Dict[str, Any]) -> Dict[str, Any]:
        angular_speed = params.get('speed', 1.0)
        angle = params.get('angle', 90)
        
        self.linear_velocity = 0.0
        self.angular_velocity = angular_speed
        duration = 1.5 
        
        self.logger.debug(f"Turning left: {angle} at {angular_speed}")
        
        return {
            'success': True,
            'duration': duration,
            'message': f'Turning left {angle}'
        }
    
    def _turn_right(self, params: Dict[str, Any]) -> Dict[str, Any]:
        angular_speed = params.get('speed', 1.0)
        angle = params.get('angle', 90)
        
        self.linear_velocity = 0.0
        self.angular_velocity = -angular_speed
        duration = 1.5 
        
        self.logger.debug(f"Turning right: {angle} at {angular_speed}")
        
        return {
            'success': True,
            'duration': duration,
            'message': f'Turning right {angle}'
        }
    
    def _stop(self) -> Dict[str, Any]:
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        self.logger.debug("Robot stopped")
        
        return {
            'success': True,
            'message': 'Robot stopped'
        }
    
    def _get_state(self) -> Dict[str, Any]:
        position = self.robot.get_position()
        orientation = self.robot.get_orientation()
        
        state = {
            'position': position,
            'orientation': orientation,
            'linear_velocity': {
                'x': float(self.linear_velocity),
                'y': 0.0,
                'z': 0.0
            },
            'angular_velocity': {
                'x': 0.0,
                'y': 0.0,
                'z': float(self.angular_velocity)
            }
        }
        
        self.logger.debug(f"State query: position={position}")
        
        return {
            'success': True,
            'state': state
        }