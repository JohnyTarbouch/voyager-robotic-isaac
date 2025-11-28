import socket
import json
import threading
from typing import Dict, Any
import time

import sys
sys.path.append('..')
from common.logger import get_server_logger, CommandLogger
from common.config import SERVER_HOST, SERVER_PORT
from manual_cmd.command_handler import CommandHandler


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
        
        # Create command handler
        self.command_handler = CommandHandler(robot_controller, self.logger)
        
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
        Process incoming command
        
        Args:
            command: Command dictionary with 'action' and 'params'
        
        Returns:
            Response dictionary with 'success' and other fields
        """
        action = command.get('action')
        params = command.get('params', {})
        
        self.logger.info(f"Command received: {action} with params: {params}")
        
        try:
            return self.command_handler.handle(action, params)
        except Exception as e:
            self.logger.error(f"Error handling command: {e}", exc_info=True)
            return {'success': False, 'error': str(e)}
    
    def update(self, dt: float):
        """
        Update robot movement (call every simulation step)
        
        Args:
            dt: Delta time since last update
        """
        self.command_handler.update(dt)