#!/usr/bin/env python3
"""
Frontier Monitor Node - Continuously monitors exploration progress
and triggers recovery when no frontiers are found for extended periods
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import math

class FrontierMonitor(Node):
    def __init__(self):
        super().__init__('frontier_monitor')
        
        # Subscribers
        self.frontier_sub = self.create_subscription(
            MarkerArray, '/explore/frontiers', self.frontier_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/explore/status', 10)
        self.recovery_trigger_pub = self.create_publisher(Bool, '/frontier_monitor/trigger_recovery', 10)
        self.coverage_pub = self.create_publisher(String, '/frontier_monitor/coverage_report', 10)
        
        # Parameters
        self.declare_parameter('monitor_frequency', 2.0)
        self.declare_parameter('no_frontier_threshold', 10.0)
        self.declare_parameter('coverage_report_interval', 60.0)
        
        self.monitor_frequency = self.get_parameter('monitor_frequency').value
        self.no_frontier_threshold = self.get_parameter('no_frontier_threshold').value
        self.coverage_report_interval = self.get_parameter('coverage_report_interval').value
        
        # State tracking
        self.frontier_count = 0
        self.last_frontier_time = time.time()
        self.last_coverage_report = time.time()
        self.current_pose = None
        self.map_data = None
        self.exploration_active = True
        
        # Coverage tracking
        self.total_map_cells = 0
        self.explored_cells = 0
        self.coverage_percentage = 0.0
        
        # Create monitoring timer
        self.monitor_timer = self.create_timer(
            1.0 / self.monitor_frequency, self.monitor_callback)
        
        self.get_logger().info('🔍 Frontier Monitor started - watching for exploration progress')

    def frontier_callback(self, msg):
        """Track frontier markers from exploration"""
        self.frontier_count = len(msg.markers)
        
        if self.frontier_count > 0:
            self.last_frontier_time = time.time()
            self.exploration_active = True
            self.get_logger().debug(f'📊 Detected {self.frontier_count} frontiers')
        else:
            self.get_logger().debug('⚠️ No frontiers detected in markers')

    def map_callback(self, msg):
        """Update map data for coverage analysis"""
        self.map_data = msg
        self.analyze_coverage()

    def pose_callback(self, msg):
        """Update current robot pose"""
        self.current_pose = msg

    def analyze_coverage(self):
        """Analyze map coverage percentage"""
        if not self.map_data:
            return
            
        self.total_map_cells = len(self.map_data.data)
        if self.total_map_cells == 0:
            return
            
        # Count explored cells (free space = 0, obstacle = 100, unknown = -1)
        free_cells = sum(1 for cell in self.map_data.data if cell == 0)
        obstacle_cells = sum(1 for cell in self.map_data.data if cell == 100)
        unknown_cells = sum(1 for cell in self.map_data.data if cell == -1)
        
        # Coverage = (free + obstacles) / total (excluding unknown)
        known_cells = free_cells + obstacle_cells
        self.explored_cells = known_cells
        
        if self.total_map_cells > 0:
            self.coverage_percentage = (known_cells / self.total_map_cells) * 100.0
        
        self.get_logger().debug(f'📈 Coverage: {self.coverage_percentage:.1f}% '
                               f'({known_cells}/{self.total_map_cells} cells)')

    def monitor_callback(self):
        """Main monitoring loop"""
        current_time = time.time()
        
        # Check for no-frontier condition
        time_since_frontiers = current_time - self.last_frontier_time
        
        if time_since_frontiers > self.no_frontier_threshold and self.exploration_active:
            self.get_logger().warn(f'🚨 No frontiers detected for {time_since_frontiers:.1f}s - '
                                  f'triggering recovery')
            
            # Publish no-frontiers status
            status_msg = String()
            status_msg.data = f'no_frontiers_for_{time_since_frontiers:.1f}s'
            self.status_pub.publish(status_msg)
            
            # Trigger recovery
            recovery_msg = Bool()
            recovery_msg.data = True
            self.recovery_trigger_pub.publish(recovery_msg)
            
            self.exploration_active = False  # Prevent repeated triggers
        
        # Periodic coverage reporting
        if current_time - self.last_coverage_report > self.coverage_report_interval:
            self.report_coverage()
            self.last_coverage_report = current_time
        
        # Check if exploration might be complete
        if self.coverage_percentage > 95.0 and time_since_frontiers > 30.0:
            self.get_logger().info(f'🎉 Exploration likely complete! Coverage: {self.coverage_percentage:.1f}%')
            
            status_msg = String()
            status_msg.data = f'exploration_complete_{self.coverage_percentage:.1f}%'
            self.status_pub.publish(status_msg)

    def report_coverage(self):
        """Report current exploration coverage"""
        if self.map_data:
            report = (f'Coverage Report: {self.coverage_percentage:.1f}% explored '
                     f'({self.explored_cells}/{self.total_map_cells} cells), '
                     f'{self.frontier_count} active frontiers')
            
            self.get_logger().info(f'📊 {report}')
            
            # Publish detailed coverage report
            coverage_msg = String()
            coverage_msg.data = (f'coverage:{self.coverage_percentage:.2f},'
                               f'explored_cells:{self.explored_cells},'
                               f'total_cells:{self.total_map_cells},'
                               f'frontiers:{self.frontier_count}')
            self.coverage_pub.publish(coverage_msg)

    def reset_monitoring(self):
        """Reset monitoring state (called externally)"""
        self.last_frontier_time = time.time()
        self.exploration_active = True
        self.get_logger().info('🔄 Frontier monitoring reset')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FrontierMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()