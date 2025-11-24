import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from client.isaac_client import IsaacSimClient
from client.skill_library import SkillLibrary
from common.logger import get_client_logger


class ManualControlCLI:
    def __init__(self):
        self.client = IsaacSimClient()
        self.skills = SkillLibrary()
        self.logger = get_client_logger()
        self.last_command = None
        
        self.logger.info("Manual control CLI initialized")
    
    def print_help(self):
        print("\n=== ROBOT Manual Control ===\n")
        print("Commands:")
        print("  forward [distance] [speed]  - Move forward")
        print("  backward [distance] [speed] - Move backward")
        print("  left [angle] [speed]        - Turn left")
        print("  right [angle] [speed]       - Turn right")
        print("  square                      - Execute square pattern")
        print("  save [name]                 - Save last command as skill")
        print("  list                        - List saved skills")
        print("  exec [skill_name]           - Execute saved skill")
        print("  delete [skill_name]         - Delete a skill")
        print("  stats                       - Show skill library stats")
        print("  state                       - Get robot state")
        print("  help                        - Show this help")
        print("  quit                        - Exit\n")
    
    def run(self):
        self.print_help()
        
        while True:
            try:
                user_input = input("Command: ").strip().split()
                
                if not user_input:
                    continue
                
                cmd = user_input[0].lower()
                
                if cmd == 'quit':
                    self.logger.info("User quit")
                    break
                
                elif cmd == 'help':
                    self.print_help()
                
                elif cmd == 'forward':
                    self._cmd_forward(user_input)
                
                elif cmd == 'backward':
                    self._cmd_backward(user_input)
                
                elif cmd == 'left':
                    self._cmd_left(user_input)
                
                elif cmd == 'right':
                    self._cmd_right(user_input)
                
                elif cmd == 'square':
                    self._cmd_square()
                
                elif cmd == 'save':
                    self._cmd_save(user_input)
                
                elif cmd == 'list':
                    self._cmd_list()
                
                elif cmd == 'exec':
                    self._cmd_exec(user_input)
                
                elif cmd == 'delete':
                    self._cmd_delete(user_input)
                
                elif cmd == 'stats':
                    self._cmd_stats()
                
                elif cmd == 'state':
                    self._cmd_state()
                
                else:
                    print(f"Unknown command: {cmd}. Type 'help' for commands.")
            
            except KeyboardInterrupt:
                self.logger.info("Interrupted by user")
                break
            
            except Exception as e:
                self.logger.error(f"Command error: {e}", exc_info=True)
    
    def _cmd_forward(self, args):
        distance = float(args[1]) if len(args) > 1 else 1.0
        speed = float(args[2]) if len(args) > 2 else 0.5
        response = self.client.move_forward(distance, speed)
        self.last_command = f"client.move_forward({distance}, {speed})"
    
    def _cmd_backward(self, args):
        distance = float(args[1]) if len(args) > 1 else 1.0
        speed = float(args[2]) if len(args) > 2 else 0.5
        response = self.client.move_backward(distance, speed)
        self.last_command = f"client.move_backward({distance}, {speed})"
    
    def _cmd_left(self, args):
        angle = float(args[1]) if len(args) > 1 else 90
        speed = float(args[2]) if len(args) > 2 else 0.3
        response = self.client.turn_left(angle, speed)
        self.last_command = f"client.turn_left({angle}, {speed})"
    
    def _cmd_right(self, args):
        angle = float(args[1]) if len(args) > 1 else 90
        speed = float(args[2]) if len(args) > 2 else 0.3
        response = self.client.turn_right(angle, speed)
        self.last_command = f"client.turn_right({angle}, {speed})"
    
    def _cmd_square(self):
        for i in range(4):
            print(f"  Side {i+1}...")
            self.client.move_forward(1.0, 0.5)
            time.sleep(0.5)
            self.client.turn_right(90, 0.3)
            time.sleep(0.5)
        self.last_command = """for i in range(4):
    client.move_forward(1.0, 0.5)
    time.sleep(0.5)
    client.turn_right(90, 0.3)
    time.sleep(0.5)"""
    
    def _cmd_save(self, args):
        if len(args) < 2:
            print("Usage: save [skill_name]")
            return
        
        skill_name = args[1]
        if self.last_command:
            if self.skills.add_skill(skill_name, self.last_command, 
                                    f"Saved skill: {skill_name}"):
                print(f"Skill '{skill_name}' saved")
            else:
                print(f"Skill '{skill_name}' already exists")
        else:
            print("No command to save")
    
    def _cmd_list(self):
        print("\n=== Skill Library ===")
        skill_list = self.skills.list_skills()
        
        if not skill_list:
            print(" No skills saved yet")
        else:
            for name, desc, success, failure in skill_list:
                print(f"  {name}: {desc} (✓{success} ✗{failure})")
        print()
    
    def _cmd_exec(self, args):
        if len(args) < 2:
            print("Usage: exec [skill_name]")
            return
        
        skill_name = args[1]
        skill = self.skills.get_skill(skill_name)
        
        if skill:
            print(f"Executing skill: {skill_name}")
            print(f"Code: {skill['code']}")
            try:
                exec_context = {
                    'client': self.client,
                    'time': time,
                    'print': print
                }
                exec(skill['code'], exec_context)
                self.skills.record_success(skill_name)
                print(f"Skill '{skill_name}' executed successfully")
            except Exception as e:
                self.skills.record_failure(skill_name)
                print(f"Skill '{skill_name}' execution failed: {e}")
        else:
            print(f"Skill '{skill_name}' not found")
    
    def _cmd_delete(self, args):
        if len(args) < 2:
            print("Usage: delete [skill_name]")
            return
        
        skill_name = args[1]
        if self.skills.delete_skill(skill_name):
            print(f"Skill '{skill_name}' deleted")
        else:
            print(f"Skill '{skill_name}' not found")
    
    def _cmd_stats(self):
        stats = self.skills.get_stats()
        print("\n=== Skill Library Statistics ===")
        print(f"  Total skills: {stats['total_skills']}")
        print(f"  Total successes: {stats['total_successes']}")
        print(f"  Total failures: {stats['total_failures']}")
        if stats['total_successes'] + stats['total_failures'] > 0:
            success_rate = stats['total_successes'] / (stats['total_successes'] + stats['total_failures']) * 100
            print(f"  Success rate: {success_rate:.1f}%")
        print()
    
    def _cmd_state(self):
        response = self.client.get_state()
        if response.get('success'):
            state = response.get('state', {})
            pos = state.get('position', {})
            print(f"\nRobot State:")
            print(f"  Position: x={pos.get('x', 0):.2f}, y={pos.get('y', 0):.2f}, z={pos.get('z', 0):.2f}")
            print()
        else:
            print(f"Failed to get state: {response.get('error')}")


def main():
    cli = ManualControlCLI()
    cli.run()


if __name__ == '__main__':
    main()
