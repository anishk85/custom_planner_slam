#!/usr/bin/env python3
"""
Exploration Controller Node
Provides simple interface to control autonomous exploration
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import termios
import tty

class ExplorationController(Node):
    def __init__(self):
        super().__init__('exploration_controller')
        
        # Publisher for exploration commands
        self.command_pub = self.create_publisher(String, 'exploration_command', 10)
        
        # Subscriber for exploration status
        self.status_sub = self.create_subscription(
            String,
            'exploration_status',
            self.status_callback,
            10
        )
        
        self.current_status = "Unknown"
        
        # Terminal settings for keyboard input
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Exploration Controller initialized')
        self.print_instructions()
        
        # Timer for keyboard input checking
        self.input_timer = self.create_timer(0.1, self.check_keyboard_input)
    
    def status_callback(self, msg: String):
        """Update current exploration status"""
        self.current_status = msg.data
        self.get_logger().info(f'Status Update: {self.current_status}')
    
    def print_instructions(self):
        """Print control instructions"""
        print("\n" + "="*60)
        print("AUTONOMOUS EXPLORATION CONTROLLER")
        print("="*60)
        print("Commands:")
        print("  's' - Start exploration")
        print("  'x' - Stop exploration")
        print("  'p' - Pause exploration (not implemented)")
        print("  'r' - Resume exploration (not implemented)")
        print("  'q' - Quit controller")
        print("  'h' - Show this help")
        print("\nCurrent Status:", self.current_status)
        print("="*60)
        print("\nPress keys to control exploration...")
    
    def check_keyboard_input(self):
        """Check for keyboard input without blocking"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            self.handle_key_press(key)
    
    def handle_key_press(self, key: str):
        """Handle keyboard input"""
        key = key.lower()
        
        if key == 's':
            self.send_command('start')
            print("→ Starting exploration...")
        elif key == 'x':
            self.send_command('stop')
            print("→ Stopping exploration...")
        elif key == 'p':
            self.send_command('pause')
            print("→ Pausing exploration... (not implemented)")
        elif key == 'r':
            self.send_command('resume')
            print("→ Resuming exploration... (not implemented)")
        elif key == 'h':
            self.print_instructions()
        elif key == 'q':
            print("→ Quitting controller...")
            self.quit_controller()
        elif key == '\x03':  # Ctrl+C
            self.quit_controller()
        else:
            print(f"Unknown command: '{key}'. Press 'h' for help.")
    
    def send_command(self, command: str):
        """Send command to exploration node"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command}')
    
    def quit_controller(self):
        """Clean shutdown"""
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
        # Send stop command before quitting
        self.send_command('stop')
        
        print("\nShutting down exploration controller...")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # Set terminal to raw mode for immediate key detection
    tty.setraw(sys.stdin.fileno())
    
    controller = ExplorationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, controller.settings)
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()