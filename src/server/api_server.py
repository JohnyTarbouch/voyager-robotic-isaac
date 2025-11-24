import socket
import json
import threading
from typing import Dict, Any, Optional
import time

import sys
sys.path.append('..')
from common.logger import get_server_logger, CommandLogger
from common.config import SERVER_HOST, SERVER_PORT


class APIServer:
    """
    Socket API server for robot control
    """
    
    def __init__(self, robot_controller, port: int = SERVER_PORT):
        """
        Initialize API server
        
        Args:
            robot_controller: Robot controller instance
            port: Server port
        """
        self.robot = robot_controller
        self.port = port
        self.running = False
        self.server_thread = None
        
        self.logger = get_server_logger()
        self.cmd_logger = CommandLogger()
        
        # Movement state
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        self.logger.info("="*70)
        self.logger.info("ROBOT API SERVER INITIALIZED")
        self.logger.info(f"Port: {self.port}")
        self.logger.info("="*70)
    
    def start(self):
        """Start the API server in a background thread"""
        self.logger.info("Starting API server.")
        
        self.running = True
        self.server_thread = threading.Thread(target=self._run_server, daemon=True)
        self.server_thread.start()
        
        self.logger.info(f"API Server running on {SERVER_HOST}:{self.port}")
        self.logger.info("Waiting for client connections.")
    
    def stop(self):
        """Stop the API server"""
        self.logger.info("Stopping API server.")
        self.running = False
        if self.server_thread:
            self.server_thread.join(timeout=2.0)
        self.logger.info("API server stopped")
    
    def _run_server(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            sock.bind((SERVER_HOST, self.port))
            sock.listen(5)
            sock.settimeout(1.0)
            
            self.logger.debug("Server socket bound and listening")
            
            while self.running:
                try:
                    client, address = sock.accept()
                    self.logger.debug(f"Client connected from {address}")
                    
                    # Receive command
                    data = client.recv(4096).decode('utf-8')
                    
                    if data:
                        start_time = time.time()
                        command = json.loads(data)
                        response = self.handle_command(command)
                        duration = time.time() - start_time
                        
                        # Log command
                        self.cmd_logger.log_command(
                            action=command.get('action'),
                            params=command.get('params', {}),
                            success=response.get('success', False),
                            duration=duration,
                            error=response.get('error')
                        )
                        
                        client.send(json.dumps(response).encode('utf-8'))
                    
                    client.close()
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    self.logger.error(f"Server error: {e}", exc_info=True)
                    
        except Exception as e:
            self.logger.error(f"Fatal server error: {e}", exc_info=True)
        finally:
            sock.close()
            self.logger.debug("Server socket closed")
    
    def handle_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process coming command
        
        Args:
            command: Command dictionary with 'action' and 'params'
        
        Returns:
            Response dictionary with 'success' and other fields
        """
        action = command.get('action')
        params = command.get('params', {})
        
        self.logger.info(f"Command received: {action} with params: {params}")
        
        try:
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
                
        except Exception as e:
            self.logger.error(f"Error handling command: {e}", exc_info=True)
            return {'success': False, 'error': str(e)}








################################################### COMMAND HANDLERS ###################################################







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
    
    def update(self, dt: float):
        """
        Update robot movement (call every simulation step)
        
        Args:
            dt: Delta time since last update
        """
        if abs(self.linear_velocity) > 0.001 or abs(self.angular_velocity) > 0.001:
            self.robot.move_by(
                forward=self.linear_velocity,
                rotation=self.angular_velocity,
                dt=dt
            )
            self.logger.debug(f"Robot updated: linear_velocity={self.linear_velocity}, angular_velocity={self.angular_velocity}, dt={dt}")
