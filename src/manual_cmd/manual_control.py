"""
Manual Robot Control Client
Send commands to the robot server interactively
"""

import socket
import json
import sys
import time
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from common.config import SERVER_HOST, SERVER_PORT
from common.logger import get_manualcmd_logger


class ManualController:
    """Interactive manual controller for the robot"""
    
    def __init__(self):
        self.logger = get_manualcmd_logger()
        self.host = SERVER_HOST
        self.port = SERVER_PORT
        
        self.logger.info("="*70)
        self.logger.info("MANUAL ROBOT CONTROLLER")
        self.logger.info(f"Connecting to: {self.host}:{self.port}")
        self.logger.info("="*70)
    
    def send_command(self, action: str, params: dict = None) -> dict:
        """
        Send command to robot server
        
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
        
        try:
            # Create socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            
            # Connect
            sock.connect((self.host, self.port))
            
            # Send command
            sock.send(json.dumps(command).encode('utf-8'))
            
            # Receive response
            response = sock.recv(4096).decode('utf-8')
            result = json.loads(response)
            
            sock.close()
            
            return result
            
        except socket.timeout:
            self.logger.error("Connection timeout")
            return {'success': False, 'error': 'Connection timeout'}
        except ConnectionRefusedError:
            self.logger.error("Connection refused - is the server running?")
            return {'success': False, 'error': 'Server not running'}
        except Exception as e:
            self.logger.error(f"Error: {e}")
            return {'success': False, 'error': str(e)}
    
    def print_menu(self):
        """Print command menu"""
        print("\n" + "="*70)
        print("ROBOT COMMANDS")
        print("="*70)
        print("Movement:")
        print("  w - Move forward 1m")
        print("  s - Move backward 1m")
        print("  a - Turn left 90°")
        print("  d - Turn right 90°")
        print("  x - STOP")
        print()
        print("Custom:")
        print("  f - Forward (custom distance)")
        print("  b - Backward (custom distance)")
        print("  l - Turn left (custom angle)")
        print("  r - Turn right (custom angle)")
        print()
        print("Info:")
        print("  p - Get position")
        print("  o - Get orientation")
        print()
        print("System:")
        print("  h - Show this menu")
        print("  q - Quit")
        print("="*70)
    
    def run_interactive(self):
        """Run interactive control loop"""
        self.print_menu()
        
        print("\nReady for commands! (press 'h' for help)")
        
        while True:
            try:
                # Get user input
                cmd = input("\nCommand: ").strip().lower()
                
                if not cmd:
                    continue
                
                # Handle commands
                if cmd == 'q':
                    print("Stopping robot...")
                    self.send_command('stop')
                    print("Goodbye!")
                    break
                
                elif cmd == 'h':
                    self.print_menu()
                
                elif cmd == 'w':
                    print("Moving forward 1m...")
                    result = self.send_command('move_forward', {
                        'distance': 1.0,
                        'speed': 0.5
                    })
                    print(f"  → {result.get('message', result)}")
                
                elif cmd == 's':
                    print("Moving backward 1m...")
                    result = self.send_command('move_backward', {
                        'distance': 1.0,
                        'speed': 0.5
                    })
                    print(f"  → {result.get('message', result)}")
                
                elif cmd == 'a':
                    print("Turning left 90°...")
                    result = self.send_command('turn_left', {
                        'angle': 90,
                        'angular_speed': 1.0
                    })
                    print(f"  → {result.get('message', result)}")
                
                elif cmd == 'd':
                    print("Turning right 90°...")
                    result = self.send_command('turn_right', {
                        'angle': 90,
                        'angular_speed': 1.0
                    })
                    print(f"  → {result.get('message', result)}")
                
                elif cmd == 'x':
                    print("STOPPING...")
                    result = self.send_command('stop')
                    print(f"  → {result.get('message', 'Stopped')}")
                
                elif cmd == 'f':
                    distance = float(input("  Distance (m): "))
                    speed = float(input("  Speed (m/s) [0.5]: ") or "0.5")
                    print(f"Moving forward {distance}m at {speed}m/s...")
                    result = self.send_command('move_forward', {
                        'distance': distance,
                        'speed': speed
                    })
                    print(f"  → {result.get('message', result)}")
                
                elif cmd == 'b':
                    distance = float(input("  Distance (m): "))
                    speed = float(input("  Speed (m/s) [0.5]: ") or "0.5")
                    print(f"Moving backward {distance}m at {speed}m/s...")
                    result = self.send_command('move_backward', {
                        'distance': distance,
                        'speed': speed
                    })
                    print(f"  → {result.get('message', result)}")
                
                elif cmd == 'l':
                    angle = float(input("  Angle (degrees): "))
                    speed = float(input("  Angular speed (rad/s) [1.0]: ") or "1.0")
                    print(f"Turning left {angle}°...")
                    result = self.send_command('turn_left', {
                        'angle': angle,
                        'angular_speed': speed
                    })
                    print(f"  → {result.get('message', result)}")
                
                elif cmd == 'r':
                    angle = float(input("  Angle (degrees): "))
                    speed = float(input("  Angular speed (rad/s) [1.0]: ") or "1.0")
                    print(f"Turning right {angle}°...")
                    result = self.send_command('turn_right', {
                        'angle': angle,
                        'angular_speed': speed
                    })
                    print(f"  → {result.get('message', result)}")
                
                elif cmd == 'p':
                    result = self.send_command('get_position')
                    if result.get('success'):
                        pos = result.get('position', {})
                        print(f"  Position: x={pos.get('x', 0):.3f}, y={pos.get('y', 0):.3f}, z={pos.get('z', 0):.3f}")
                    else:
                        print(f"  Error: {result.get('error')}")
                
                elif cmd == 'o':
                    result = self.send_command('get_orientation')
                    if result.get('success'):
                        ori = result.get('orientation', {})
                        print(f"  Orientation: w={ori.get('w', 1):.3f}, x={ori.get('x', 0):.3f}, y={ori.get('y', 0):.3f}, z={ori.get('z', 0):.3f}")
                    else:
                        print(f"  Error: {result.get('error')}")
                
                else:
                    print(f"Unknown command: '{cmd}' (press 'h' for help)")
                
            except KeyboardInterrupt:
                print("\n\nStopping robot...")
                self.send_command('stop')
                print("Goodbye!")
                break
            except ValueError as e:
                print(f"Invalid input: {e}")
            except Exception as e:
                self.logger.error(f"Error: {e}", exc_info=True)
                print(f"Error: {e}")


def main():
    """Main entry point"""
    controller = ManualController()
    
    # Test connection
    print("\nTesting connection to server...")
    result = controller.send_command('get_position')
    
    if result.get('success'):
        print("✓ Connected successfully!")
        pos = result.get('position', {})
        print(f"✓ Robot position: x={pos.get('x', 0):.2f}, y={pos.get('y', 0):.2f}, z={pos.get('z', 0):.2f}")
    else:
        print(f"✗ Connection failed: {result.get('error')}")
        print("\nMake sure the server is running:")
        print("  python.bat C:\\isaacsim\\standalone_examples\\voyager-robotic-isaac\\src\\server\\main.py")
        return
    
    # Run interactive mode
    controller.run_interactive()


if __name__ == '__main__':
    main()