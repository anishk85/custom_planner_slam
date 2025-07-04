#!/usr/bin/env python3
"""
Autonomous Explorer Node - Optimized for Small House
Main node that coordinates frontier detection, goal selection, and navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

import numpy as np
import time
from enum import Enum
from typing import List, Optional

from .frontier_detector import FrontierDetector

class ExplorationState(Enum):
    IDLE = 0
    DETECTING_FRONTIERS = 1
    SELECTING_GOAL = 2
    NAVIGATING = 3
    EXPLORATION_COMPLETE = 4
    ERROR = 5

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Parameters - Optimized for small house
        self.declare_parameter('min_frontier_size', 3)         # Smaller for small house
        self.declare_parameter('min_frontier_distance', 0.3)   # Closer for small house
        self.declare_parameter('goal_timeout', 25.0)           # Shorter timeout
        self.declare_parameter('exploration_timeout', 300.0)   # 5 minutes
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        # Get parameters
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.min_frontier_distance = self.get_parameter('min_frontier_distance').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.exploration_timeout = self.get_parameter('exploration_timeout').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # State management
        self.state = ExplorationState.IDLE
        self.current_map = None
        self.current_frontiers = []
        self.current_goal = None
        self.exploration_start_time = None
        self.failed_goals = []  # Track failed goals to avoid repeating
        
        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize frontier detector
        self.frontier_detector = FrontierDetector(min_frontier_size=self.min_frontier_size)
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'exploration_status', 10)
        self.frontier_pub = self.create_publisher(MarkerArray, 'frontiers_markers', 10)
        self.goal_pub = self.create_publisher(Marker, 'current_goal_marker', 10)
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            'map', 
            self.map_callback, 
            10,
            callback_group=self.callback_group
        )
        
        self.cmd_sub = self.create_subscription(
            String,
            'exploration_command',
            self.command_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Navigation action client
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Main exploration timer
        self.exploration_timer = self.create_timer(
            1.0, 
            self.exploration_loop,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Autonomous Explorer initialized for Small House')
        self.publish_status('Initialized - Ready to start exploration')
    
    def map_callback(self, msg: OccupancyGrid):
        """Update current map data"""
        self.current_map = msg
    
    def command_callback(self, msg: String):
        """Handle exploration commands"""
        command = msg.data.lower()
        
        if command == 'start':
            self.start_exploration()
        elif command == 'stop':
            self.stop_exploration()
        elif command == 'pause':
            self.pause_exploration()
        elif command == 'resume':
            self.resume_exploration()
        else:
            self.get_logger().warn(f'Unknown command: {command}')
    
    def start_exploration(self):
        """Start autonomous exploration"""
        if self.state == ExplorationState.IDLE:
            self.state = ExplorationState.DETECTING_FRONTIERS
            self.exploration_start_time = time.time()
            self.failed_goals = []  # Reset failed goals
            self.get_logger().info('Starting autonomous exploration of small house')
            self.publish_status('Exploration started')
        else:
            self.get_logger().warn('Exploration already running or in error state')
    
    def stop_exploration(self):
        """Stop autonomous exploration"""
        if self.state != ExplorationState.IDLE:
            # Cancel current navigation if active
            if self.state == ExplorationState.NAVIGATING and self.current_goal:
                self.nav_client.cancel_all_goals()
            
            self.state = ExplorationState.IDLE
            self.current_goal = None
            self.get_logger().info('Exploration stopped')
            self.publish_status('Exploration stopped')
    
    def pause_exploration(self):
        """Pause exploration (not implemented yet)"""
        self.get_logger().info('Pause not implemented yet')
    
    def resume_exploration(self):
        """Resume exploration (not implemented yet)"""
        self.get_logger().info('Resume not implemented yet')
    
    def exploration_loop(self):
        """Main exploration state machine"""
        if self.state == ExplorationState.IDLE:
            return
        
        # Check for exploration timeout
        if (self.exploration_start_time and 
            time.time() - self.exploration_start_time > self.exploration_timeout):
            self.get_logger().info('Exploration timeout reached')
            self.state = ExplorationState.EXPLORATION_COMPLETE
            self.publish_status('Exploration completed - Timeout reached')
            return
        
        # State machine
        if self.state == ExplorationState.DETECTING_FRONTIERS:
            self.detect_frontiers()
        elif self.state == ExplorationState.SELECTING_GOAL:
            self.select_goal()
        elif self.state == ExplorationState.NAVIGATING:
            self.check_navigation_status()
        elif self.state == ExplorationState.EXPLORATION_COMPLETE:
            self.get_logger().info('Small house exploration completed successfully')
            self.publish_status('Small house exploration completed successfully')
            self.state = ExplorationState.IDLE
    
    def detect_frontiers(self):
        """Detect frontiers in current map"""
        if self.current_map is None:
            self.get_logger().warn('No map available for frontier detection')
            return
        
        try:
            # Get robot position
            robot_pos = self.get_robot_position()
            if robot_pos is None:
                self.get_logger().warn('Cannot get robot position')
                return
            
            # Detect frontiers
            frontier_regions = self.frontier_detector.detect_frontiers(self.current_map)
            frontier_centroids = self.frontier_detector.get_frontier_centroids(frontier_regions)
            
            # Filter frontiers by distance and failed goals
            filtered_frontiers = self.frontier_detector.filter_frontiers_by_distance(
                frontier_centroids, robot_pos, self.min_frontier_distance
            )
            
            # Remove previously failed goals
            self.current_frontiers = self.filter_failed_goals(filtered_frontiers)
            
            # Visualize frontiers
            self.visualize_frontiers(self.current_frontiers)
            
            self.get_logger().info(f'Detected {len(self.current_frontiers)} valid frontiers')
            
            if len(self.current_frontiers) > 0:
                self.state = ExplorationState.SELECTING_GOAL
            else:
                self.state = ExplorationState.EXPLORATION_COMPLETE
                self.publish_status('No more frontiers - Small house exploration complete')
        
        except Exception as e:
            self.get_logger().error(f'Error in frontier detection: {str(e)}')
            self.state = ExplorationState.ERROR
    
    def filter_failed_goals(self, frontiers: List[Point]) -> List[Point]:
        """Filter out previously failed goals"""
        filtered = []
        for frontier in frontiers:
            # Check if this frontier is too close to any failed goal
            is_failed = False
            for failed_goal in self.failed_goals:
                distance = np.sqrt(
                    (frontier.x - failed_goal.x) ** 2 + 
                    (frontier.y - failed_goal.y) ** 2
                )
                if distance < 0.5:  # 0.5m threshold
                    is_failed = True
                    break
            
            if not is_failed:
                filtered.append(frontier)
        
        return filtered
    
    def select_goal(self):
        """Select next frontier to explore - optimized for small house"""
        if not self.current_frontiers:
            self.state = ExplorationState.EXPLORATION_COMPLETE
            return
        
        try:
            robot_pos = self.get_robot_position()
            if robot_pos is None:
                return
            
            # Strategy for small house: select nearest accessible frontier
            best_frontier = self.select_best_frontier_small_house(self.current_frontiers, robot_pos)
            
            if best_frontier:
                self.current_goal = best_frontier
                self.send_navigation_goal(best_frontier)
                self.state = ExplorationState.NAVIGATING
                self.publish_status(f'Navigating to frontier at ({best_frontier.x:.2f}, {best_frontier.y:.2f})')
            else:
                self.state = ExplorationState.EXPLORATION_COMPLETE
        
        except Exception as e:
            self.get_logger().error(f'Error in goal selection: {str(e)}')
            self.state = ExplorationState.ERROR
    
    def select_best_frontier_small_house(self, frontiers: List[Point], robot_pos: Point) -> Optional[Point]:
        """Select the best frontier for small house exploration"""
        if not frontiers:
            return None
        
        # For small house, prefer frontiers that are:
        # 1. Not too far (within 3 meters)
        # 2. Not too close (at least min_frontier_distance)
        # 3. Closest among valid options
        
        valid_frontiers = []
        for frontier in frontiers:
            distance = np.sqrt(
                (frontier.x - robot_pos.x) ** 2 + 
                (frontier.y - robot_pos.y) ** 2
            )
            
            # Filter by distance range suitable for small house
            if self.min_frontier_distance <= distance <= 3.0:
                valid_frontiers.append((frontier, distance))
        
        if not valid_frontiers:
            # If no valid frontiers, just take the closest one
            return self.select_nearest_frontier(frontiers, robot_pos)
        
        # Sort by distance and return closest
        valid_frontiers.sort(key=lambda x: x[1])
        return valid_frontiers[0][0]
    
    def select_nearest_frontier(self, frontiers: List[Point], robot_pos: Point) -> Optional[Point]:
        """Select the nearest frontier to the robot"""
        if not frontiers:
            return None
        
        min_distance = float('inf')
        best_frontier = None
        
        for frontier in frontiers:
            distance = np.sqrt(
                (frontier.x - robot_pos.x) ** 2 + 
                (frontier.y - robot_pos.y) ** 2
            )
            
            if distance < min_distance:
                min_distance = distance
                best_frontier = frontier
        
        return best_frontier
    
    def send_navigation_goal(self, goal_point: Point):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = goal_point
        goal_msg.pose.pose.orientation.w = 1.0  # No specific orientation
        
        self.get_logger().info(f'Sending navigation goal: ({goal_point.x:.2f}, {goal_point.y:.2f})')
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            self.state = ExplorationState.ERROR
            return
        
        # Send goal
        self.nav_future = self.nav_client.send_goal_async(goal_msg)
        self.nav_start_time = time.time()
        
        # Visualize current goal
        self.visualize_goal(goal_point)
    
    def check_navigation_status(self):
        """Check navigation progress"""
        # Check for navigation timeout
        if time.time() - self.nav_start_time > self.goal_timeout:
            self.get_logger().warn('Navigation timeout - adding to failed goals and selecting new goal')
            self.nav_client.cancel_all_goals()
            if self.current_goal:
                self.failed_goals.append(self.current_goal)
            self.state = ExplorationState.DETECTING_FRONTIERS
            return
        
        # Check if navigation is complete
        if hasattr(self, 'nav_future') and self.nav_future.done():
            try:
                goal_handle = self.nav_future.result()
                if goal_handle.accepted:
                    result_future = goal_handle.get_result_async()
                    if result_future.done():
                        result = result_future.result()
                        if result.status == 4:  # SUCCEEDED
                            self.get_logger().info('Navigation succeeded - looking for new frontiers')
                            self.state = ExplorationState.DETECTING_FRONTIERS
                        else:
                            self.get_logger().warn(f'Navigation failed with status: {result.status}')
                            if self.current_goal:
                                self.failed_goals.append(self.current_goal)
                            self.state = ExplorationState.DETECTING_FRONTIERS
                else:
                    self.get_logger().warn('Navigation goal rejected')
                    if self.current_goal:
                        self.failed_goals.append(self.current_goal)
                    self.state = ExplorationState.DETECTING_FRONTIERS
            except Exception as e:
                self.get_logger().error(f'Navigation error: {str(e)}')
                if self.current_goal:
                    self.failed_goals.append(self.current_goal)
                self.state = ExplorationState.DETECTING_FRONTIERS
    
    def get_robot_position(self) -> Optional[Point]:
        """Get current robot position in map frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.robot_frame, 
                rclpy.time.Time()
            )
            
            point = Point()
            point.x = transform.transform.translation.x
            point.y = transform.transform.translation.y
            point.z = 0.0
            
            return point
        
        except TransformException as e:
            self.get_logger().warn(f'Could not get robot position: {str(e)}')
            return None
    
    def visualize_frontiers(self, frontiers: List[Point]):
        """Visualize frontiers as markers"""
        marker_array = MarkerArray()
        
        for i, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = frontier
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.15  # Smaller for small house
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.frontier_pub.publish(marker_array)
    
    def visualize_goal(self, goal: Point):
        """Visualize current goal as marker"""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'current_goal'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = goal
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3  # Smaller for small house
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.goal_pub.publish(marker)
    
    def publish_status(self, status: str):
        """Publish exploration status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {status}')

def main(args=None):
    rclpy.init(args=args)
    
    explorer = AutonomousExplorer()
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(explorer, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()