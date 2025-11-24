import socket
import json
import time
from typing import Dict, Any, Optional

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from common.config import SERVER_HOST, SERVER_PORT
from common.logger import get_manualcmd_logger


class IsaacSimClient:
    """
    Client to communicate with Isaac Sim
    """
    
    def __init__(self, host: str = SERVER_HOST, port: int = SERVER_PORT):
        self.host = host
        self.port = port
        self.logger = get_manualcmd_logger()
        
        self.logger.info(f"Isaac Sim Client initialized (server: {host}:{port})")
    
    def send_command(self, action: str, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Send command to Isaac Sim server
        
        Args:
            action: Command action
            params: Command parameters
        
        Returns:
            Response dictionary
        """
        if params is None:
            params = {}
        
        command = {
            'action': action,
            'params': params
        }
        
        self.logger.debug(f"Sending command: {action} with params: {params}")
        
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.settimeout(5.0)  # 5 second timeout
            client_socket.connect((self.host, self.port))
            
            # Send cmd
            client_socket.send(json.dumps(command).encode('utf-8'))
            
            # Receive resp
            response = client_socket.recv(4096).decode('utf-8')
            client_socket.close()
            
            result = json.loads(response)
            self.logger.debug(f"Received response: {result}")
            
            return result
        
        except socket.timeout:
            error_msg = "Connection timeout"
            self.logger.error(error_msg)
            return {'success': False, 'error': error_msg}
        
        except ConnectionRefusedError:
            error_msg = "Connection refused"
            self.logger.error(error_msg)
            return {'success': False, 'error': error_msg}
        
        except Exception as e:
            error_msg = f"Communication error: {str(e)}"
            self.logger.error(error_msg, exc_info=True)
            return {'success': False, 'error': error_msg}
    
    def move_forward(self, distance: float = 1.0, speed: float = 0.5) -> Dict[str, Any]:
        """
        Move robot forward
        
        Args:
            distance: Distance in meters
            speed
        
        Returns:
            Response dictionary
        """
        self.logger.info(f"Move forward: {distance} at {speed}")
        response = self.send_command('move_forward', {
            'distance': distance,
            'speed': speed
        })
        
        if response.get('success'):
            duration = response.get('duration', 0)
            time.sleep(duration)
            self.stop()
        
        return response
    
    def move_backward(self, distance: float = 1.0, speed: float = 0.5) -> Dict[str, Any]:
        self.logger.info(f"Move backward: {distance} at {speed}")
        response = self.send_command('move_backward', {
            'distance': distance,
            'speed': speed
        })
        
        if response.get('success'):
            duration = response.get('duration', 0)
            time.sleep(duration)
            self.stop()
        
        return response
    
    def turn_left(self, angle: float = 90, speed: float = 0.3) -> Dict[str, Any]:
        self.logger.info(f"Turn left: {angle} at {speed}")
        response = self.send_command('turn_left', {
            'angle': angle,
            'speed': speed
        })
        
        if response.get('success'):
            duration = response.get('duration', 0)
            time.sleep(duration)
            self.stop()
        
        return response
    
    def turn_right(self, angle: float = 90, speed: float = 0.3) -> Dict[str, Any]:
        self.logger.info(f"Turn right: {angle} at {speed}") 
        response = self.send_command('turn_right', {
            'angle': angle,
            'speed': speed
        })
        
        if response.get('success'):
            duration = response.get('duration', 0)
            time.sleep(duration)
            self.stop()
        
        return response
    
    def stop(self) -> Dict[str, Any]:
        self.logger.debug("Stop command")
        return self.send_command('stop')
    
    def get_state(self) -> Dict[str, Any]:
        self.logger.debug("Get state command")
        return self.send_command('get_state')
    
    def execute_code(self, code: str) -> Dict[str, Any]:
        self.logger.info(f"Execute code: {code[:100]}...")
        return self.send_command('execute_code', {'code': code})
