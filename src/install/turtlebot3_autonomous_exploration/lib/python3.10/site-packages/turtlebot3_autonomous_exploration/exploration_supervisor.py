#!/usr/bin/env python3
"""
Exploration Supervisor Node - Supervises the entire exploration process
and restarts exploration when it stops prematurely
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import time
import subprocess
import threading

class ExplorationSupervisor(Node):
    def __init__(self):
        super().__init__('exploration_supervisor')
        
        # Subscribers
        self.explore_status_sub = self.create_subscription(
            String, '/explore/status', self.explore_status_callback, 10)
        self.recovery_status_sub = self.create_subscription(
            String, '/exploration_recovery/status', self.recovery_status_callback, 10)
        self.coverage_sub = self.create_subscription(
            String, '/frontier_monitor/coverage_report', self.coverage_callback, 10)
        
        # Publishers
        self.supervisor_status_pub = self.create_publisher(String, '/exploration_supervisor/status', 10)
        self.restart_trigger_pub = self.create_publisher(Bool, '/exploration_supervisor/restart', 10)
        
        # Service clients for lifecycle management
        self.explore_change_state_client = self.create_client(
            ChangeState, '/explore_node/change_state')
        
        # Parameters
        self.declare_parameter('supervision_frequency', 1.0)
        self.declare_parameter('restart_exploration_delay', 5.0)
        self.declare_parameter('max_restart_attempts', 10)
        self.declare_parameter('exploration_timeout', 1800.0)  # 30 minutes
        
        self.supervision_frequency = self.get_parameter('supervision_frequency').value
        self.restart_delay = self.get_parameter('restart_exploration_delay').value
        self.max_restart_attempts = self.get_parameter('max_restart_attempts').value
        self.exploration_timeout = self.get_parameter('exploration_timeout').value
        
        # State tracking
        self.exploration_active = True
        self.exploration_start_time = time.time()
        self.last_exploration_activity = time.time()
        self.restart_attempts = 0
        self.current_coverage = 0.0
        self.exploration_stopped_time = None
        
        # Status tracking
        self.last_explore_status = "unknown"
        self.last_recovery_status = "unknown"
        self.supervisor_active = True
        
        # Create supervision timer
        self.supervision_timer = self.create_timer(
            1.0 / self.supervision_frequency, self.supervision_callback)
        
        self.get_logger().info('👁️ Exploration Supervisor started - monitoring exploration process')

    def explore_status_callback(self, msg):
        """Monitor exploration node status"""
        status = msg.data.lower()
        self.last_explore_status = status
        
        if 'stopping' in status or 'stopped' in status or 'no frontiers' in status:
            if self.exploration_active:
                self.get_logger().warn(f'⚠️ Exploration stopped: {status}')
                self.exploration_active = False
                self.exploration_stopped_time = time.time()
        elif 'exploring' in status or 'moving' in status or 'frontier' in status:
            if not self.exploration_active:
                self.get_logger().info(f'✅ Exploration resumed: {status}')
                self.exploration_active = True
                self.exploration_stopped_time = None
            self.last_exploration_activity = time.time()

    def recovery_status_callback(self, msg):
        """Monitor recovery system status"""
        self.last_recovery_status = msg.data
        
        if 'recovery_completed' in msg.data:
            self.get_logger().info('🔧 Recovery completed, exploration should resume')
            self.last_exploration_activity = time.time()
        elif 'manual_intervention_required' in msg.data:
            self.get_logger().error('🆘 Manual intervention required - stopping supervision')
            self.supervisor_active = False

    def coverage_callback(self, msg):
        """Monitor coverage progress"""
        try:
            # Parse coverage data: "coverage:XX.XX,explored_cells:YYYY,..."
            parts = msg.data.split(',')
            for part in parts:
                if part.startswith('coverage:'):
                    self.current_coverage = float(part.split(':')[1])
                    break
        except (ValueError, IndexError):
            self.get_logger().warn(f'⚠️ Could not parse coverage data: {msg.data}')

    def supervision_callback(self):
        """Main supervision loop"""
        if not self.supervisor_active:
            return
            
        current_time = time.time()
        
        # Check for exploration timeout
        if current_time - self.exploration_start_time > self.exploration_timeout:
            self.get_logger().warn(f'⏰ Exploration timeout reached ({self.exploration_timeout}s)')
            self.handle_exploration_timeout()
            return
        
        # Check if exploration has been stopped for too long
        if (not self.exploration_active and 
            self.exploration_stopped_time and 
            current_time - self.exploration_stopped_time > self.restart_delay):
            
            # Only restart if coverage is incomplete
            if self.current_coverage < 95.0:
                self.get_logger().warn(f'🔄 Exploration stopped for {current_time - self.exploration_stopped_time:.1f}s '
                                      f'with {self.current_coverage:.1f}% coverage - attempting restart')
                self.restart_exploration()
            else:
                self.get_logger().info(f'✅ Exploration stopped but coverage is {self.current_coverage:.1f}% - '
                                      'considering complete')
                self.supervisor_active = False
        
        # Publish supervisor status
        self.publish_supervisor_status()

    def restart_exploration(self):
        """Attempt to restart the exploration process"""
        if self.restart_attempts >= self.max_restart_attempts:
            self.get_logger().error(f'❌ Maximum restart attempts ({self.max_restart_attempts}) reached')
            self.supervisor_active = False
            return
        
        self.restart_attempts += 1
        self.get_logger().info(f'🔄 Restart attempt {self.restart_attempts}/{self.max_restart_attempts}')
        
        # Method 1: Try to restart via ROS2 launch
        restart_thread = threading.Thread(target=self.restart_exploration_async)
        restart_thread.daemon = True
        restart_thread.start()
        
        # Reset timers
        self.exploration_stopped_time = None
        self.last_exploration_activity = time.time()

    def restart_exploration_async(self):
        """Restart exploration in a separate thread"""
        try:
            # Method 1: Try ROS2 service call to restart explore node
            self.get_logger().info('🔧 Attempting to restart exploration via service call')
            
            if self.explore_change_state_client.wait_for_service(timeout_sec=5.0):
                request = ChangeState.Request()
                request.transition = Transition()
                request.transition.id = Transition.TRANSITION_DEACTIVATE
                
                # Deactivate first
                future = self.explore_change_state_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() and future.result().success:
                    time.sleep(2.0)  # Wait a moment
                    
                    # Then reactivate
                    request.transition.id = Transition.TRANSITION_ACTIVATE
                    future = self.explore_change_state_client.call_async(request)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                    
                    if future.result() and future.result().success:
                        self.get_logger().info('✅ Exploration restarted via lifecycle')
                        return
            
            # Method 2: Try to restart via command line
            self.get_logger().info('🔧 Attempting to restart via rosnode restart')
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=10)
            
            if '/explore_node' in result.stdout:
                # Kill and restart the node
                subprocess.run(['ros2', 'lifecycle', 'set', '/explore_node', 'deactivate'], 
                             timeout=10)
                time.sleep(2.0)
                subprocess.run(['ros2', 'lifecycle', 'set', '/explore_node', 'activate'], 
                             timeout=10)
                self.get_logger().info('✅ Exploration restarted via command line')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to restart exploration: {str(e)}')
        
        # Trigger restart signal regardless
        restart_msg = Bool()
        restart_msg.data = True
        self.restart_trigger_pub.publish(restart_msg)

    def handle_exploration_timeout(self):
        """Handle exploration timeout"""
        if self.current_coverage >= 90.0:
            self.get_logger().info(f'✅ Exploration timeout but coverage is {self.current_coverage:.1f}% - '
                                  'considering successful')
        else:
            self.get_logger().warn(f'⚠️ Exploration timeout with only {self.current_coverage:.1f}% coverage')
        
        self.supervisor_active = False

    def publish_supervisor_status(self):
        """Publish current supervisor status"""
        current_time = time.time()
        
        status_parts = [
            f'active:{self.supervisor_active}',
            f'exploration_active:{self.exploration_active}',
            f'coverage:{self.current_coverage:.1f}%',
            f'restart_attempts:{self.restart_attempts}',
            f'runtime:{current_time - self.exploration_start_time:.0f}s'
        ]
        
        status_msg = String()
        status_msg.data = ','.join(status_parts)
        self.supervisor_status_pub.publish(status_msg)

    def shutdown_supervisor(self):
        """Gracefully shutdown the supervisor"""
        self.get_logger().info('🛑 Exploration Supervisor shutting down')
        self.supervisor_active = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ExplorationSupervisor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.shutdown_supervisor()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()