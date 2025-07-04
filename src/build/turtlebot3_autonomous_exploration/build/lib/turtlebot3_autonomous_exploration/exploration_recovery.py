#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy
import random
import time
import math
from collections import deque

class ExplorationRecovery(Node):
    def __init__(self):
        super().__init__('exploration_recovery')
        
        # QoS profiles for reliable communication
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, qos_profile)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.explore_status_sub = self.create_subscription(
            String, '/explore/status', self.explore_status_callback, qos_profile)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', qos_profile)
        self.skip_pub = self.create_publisher(String, '/explore/skip_frontier', qos_profile)
        self.status_pub = self.create_publisher(String, '/exploration_recovery/status', qos_profile)
        self.recovery_active_pub = self.create_publisher(Bool, '/exploration_recovery/active', qos_profile)
        self.explore_resume_pub = self.create_publisher(String, '/explore/resume', qos_profile)
        self.frontier_search_pub = self.create_publisher(String, '/explore/force_search', qos_profile)
        
        # State variables
        self.pose_history = deque(maxlen=20)  # Keep last 20 poses
        self.current_pose = None
        self.map_data = None
        self.last_recovery_time = time.time()
        self.recovery_attempts = 0
        self.max_recovery_attempts = 7  # Increased for more thorough exploration
        self.is_recovering = False
        self.stuck_threshold_distance = 0.15  # meters
        self.stuck_threshold_time = 25.0  # seconds
        self.recovery_cooldown = 45.0  # seconds between recovery attempts
        
        # Exploration completion tracking
        self.exploration_stopped = False
        self.no_frontiers_count = 0
        self.last_frontier_time = time.time()
        self.exploration_positions = []  # Track where we've been
        self.coverage_check_positions = []  # Positions to revisit for coverage
        
        # Coverage analysis
        self.map_explored_ratio = 0.0
        self.last_coverage_check = time.time()
        self.coverage_threshold = 0.85  # 85% explored before considering complete
        
        # Circular motion detection
        self.orientation_history = deque(maxlen=10)
        self.total_rotation = 0.0
        
        # Recovery behavior weights (adjusted based on success)
        self.recovery_weights = {
            'force_frontier_search': 1.0,
            'coverage_check': 1.0,
            'backtrack_far': 0.9,
            'systematic_search': 0.8,
            'spin_and_scan': 0.7,
            'random_walk': 0.6,
            'skip_frontier': 0.4,
            'emergency_stop': 0.1
        }
        
        self.get_logger().info('🚀 Enhanced ExplorationRecovery node started.')
        self.publish_status('initialized')

    def pose_callback(self, msg):
        """Process incoming pose updates and detect stuck conditions"""
        current_time = time.time()
        pos = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Store current pose
        self.current_pose = msg
        
        # Calculate yaw from quaternion for circular motion detection
        yaw = self.quaternion_to_yaw(orientation)
        self.orientation_history.append((yaw, current_time))
        
        # Store pose with timestamp and track exploration positions
        self.pose_history.append((pos.x, pos.y, current_time))
        
        # Track unique positions for coverage analysis
        if not self.exploration_positions or \
           math.sqrt((pos.x - self.exploration_positions[-1][0])**2 + 
                    (pos.y - self.exploration_positions[-1][1])**2) > 0.5:
            self.exploration_positions.append((pos.x, pos.y, current_time))
        
        # Detect various stuck conditions
        if not self.is_recovering and len(self.pose_history) >= 3:
            if self.detect_stuck_linear() or self.detect_stuck_circular():
                self.initiate_recovery()

    def map_callback(self, msg):
        """Store map data for path planning assistance"""
        self.map_data = msg
        
        # Periodically analyze coverage
        current_time = time.time()
        if current_time - self.last_coverage_check > 30.0:  # Check every 30 seconds
            self.analyze_map_coverage()
            self.last_coverage_check = current_time

    def explore_status_callback(self, msg):
        """Monitor exploration status and trigger recovery when needed"""
        status = msg.data.lower()
        current_time = time.time()
        
        if 'no frontiers' in status or 'exploration stopped' in status or 'stopping' in status:
            self.no_frontiers_count += 1
            self.get_logger().warn(f'🚨 No frontiers detected (count: {self.no_frontiers_count})')
            
            # If exploration has stopped but coverage is incomplete, trigger recovery
            if not self.exploration_stopped:
                self.exploration_stopped = True
                self.get_logger().warn('🔍 Exploration reported complete, checking coverage...')
                
                # Trigger comprehensive recovery to ensure complete exploration
                if self.map_explored_ratio < self.coverage_threshold:
                    self.get_logger().warn(f'📊 Coverage only {self.map_explored_ratio:.1%}, initiating deep exploration recovery')
                    self.initiate_no_frontier_recovery()
                else:
                    self.get_logger().info(f'✅ Exploration truly complete! Coverage: {self.map_explored_ratio:.1%}')
        else:
            # Reset counters if exploration is active
            self.no_frontiers_count = 0
            self.exploration_stopped = False
            self.last_frontier_time = current_time

    def analyze_map_coverage(self):
        """Analyze how much of the map has been explored"""
        if not self.map_data:
            return
            
        total_cells = len(self.map_data.data)
        if total_cells == 0:
            return
            
        # Count known cells (not unknown = -1)
        known_cells = sum(1 for cell in self.map_data.data if cell != -1)
        unknown_cells = total_cells - known_cells
        
        self.map_explored_ratio = known_cells / total_cells if total_cells > 0 else 0.0
        
        self.get_logger().info(f'📊 Map coverage analysis: {self.map_explored_ratio:.1%} explored '
                              f'({known_cells}/{total_cells} cells, {unknown_cells} unknown)')

    def initiate_no_frontier_recovery(self):
        """Special recovery when no frontiers are found but exploration seems incomplete"""
        self.get_logger().warn('🔄 No frontiers found but exploration incomplete - starting deep recovery')
        self.is_recovering = True
        self.recovery_attempts = 0  # Reset for no-frontier recovery
        self.last_recovery_time = time.time()
        
        self.publish_status('no_frontiers_deep_recovery_initiated')
        recovery_msg = Bool()
        recovery_msg.data = True
        self.recovery_active_pub.publish(recovery_msg)
        
        # Start with forced frontier search
        self.execute_no_frontier_recovery()

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw

    def detect_stuck_linear(self):
        """Detect if robot is stuck (not moving forward)"""
        if len(self.pose_history) < 3:
            return False
            
        # Check movement over time window
        start_pose = self.pose_history[0]
        end_pose = self.pose_history[-1]
        
        x0, y0, t0 = start_pose
        x1, y1, t1 = end_pose
        
        distance_moved = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        time_elapsed = t1 - t0
        
        # Check if stuck based on distance and time thresholds
        is_stuck = (distance_moved < self.stuck_threshold_distance and 
                   time_elapsed > self.stuck_threshold_time and
                   (time.time() - self.last_recovery_time) > self.recovery_cooldown)
        
        if is_stuck:
            self.get_logger().warn(f'🔍 Linear stuck detected: moved {distance_moved:.3f}m in {time_elapsed:.1f}s')
        
        return is_stuck

    def detect_stuck_circular(self):
        """Detect if robot is going in circles"""
        if len(self.orientation_history) < 5:
            return False
            
        # Calculate total rotation in recent history
        total_rotation = 0.0
        for i in range(1, len(self.orientation_history)):
            prev_yaw, _ = self.orientation_history[i-1]
            curr_yaw, _ = self.orientation_history[i]
            
            # Handle angle wrapping
            diff = curr_yaw - prev_yaw
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
                
            total_rotation += abs(diff)
        
        # Check if robot has rotated significantly without moving
        recent_poses = list(self.pose_history)[-5:]  # Last 5 poses
        if len(recent_poses) >= 2:
            start_x, start_y, _ = recent_poses[0]
            end_x, end_y, _ = recent_poses[-1]
            linear_distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
            
            # Circular motion detected: significant rotation with minimal linear movement
            is_circular = (total_rotation > math.pi and linear_distance < 0.3 and
                          (time.time() - self.last_recovery_time) > self.recovery_cooldown)
            
            if is_circular:
                self.get_logger().warn(f'🔄 Circular motion detected: {total_rotation:.2f} rad rotation, {linear_distance:.2f}m displacement')
            
            return is_circular
        
        return False

    def initiate_recovery(self):
        """Start the recovery process"""
        self.get_logger().warn('🤖 Robot stuck detected! Initiating recovery sequence...')
        self.is_recovering = True
        self.recovery_attempts += 1
        self.last_recovery_time = time.time()
        
        # Publish recovery status
        self.publish_status(f'recovery_initiated_attempt_{self.recovery_attempts}')
        recovery_msg = Bool()
        recovery_msg.data = True
        self.recovery_active_pub.publish(recovery_msg)
        
        # Select and execute recovery behavior
        self.execute_recovery_behavior()

    def execute_recovery_behavior(self):
        """Choose and execute appropriate recovery behavior"""
        if self.recovery_attempts > self.max_recovery_attempts:
            self.get_logger().error('❌ Maximum recovery attempts reached. Requesting manual intervention.')
            self.publish_status('manual_intervention_required')
            self.request_manual_intervention()
            return

        # Select recovery behavior based on attempt number and context
        behaviors = ['backtrack', 'spin_and_scan', 'random_walk', 'systematic_search', 
                    'coverage_check', 'skip_frontier', 'emergency_stop']
        
        if self.recovery_attempts < len(behaviors):
            behavior = behaviors[self.recovery_attempts]
        else:
            behavior = 'emergency_stop'
        
        self.get_logger().info(f'🔧 Executing recovery behavior: {behavior}')
        
        # Execute the selected behavior
        if behavior == 'backtrack':
            self.recovery_backtrack()
        elif behavior == 'spin_and_scan':
            self.recovery_spin_and_scan()
        elif behavior == 'random_walk':
            self.recovery_random_walk()
        elif behavior == 'systematic_search':
            self.recovery_systematic_search()
        elif behavior == 'coverage_check':
            self.recovery_coverage_check()
        elif behavior == 'skip_frontier':
            self.recovery_skip_frontier()
        elif behavior == 'emergency_stop':
            self.recovery_emergency_stop()

    def execute_no_frontier_recovery(self):
        """Special recovery sequence when no frontiers are found"""
        if self.recovery_attempts > self.max_recovery_attempts:
            self.get_logger().info('🏁 All recovery attempts exhausted. Exploration truly complete.')
            self.publish_status('exploration_genuinely_complete')
            self.complete_recovery()
            return

        # Aggressive recovery sequence for no-frontier situations
        behaviors = ['force_frontier_search', 'coverage_check', 'backtrack_far', 
                    'systematic_search', 'spin_and_scan', 'random_walk', 'emergency_stop']
        
        if self.recovery_attempts < len(behaviors):
            behavior = behaviors[self.recovery_attempts]
        else:
            behavior = 'emergency_stop'
        
        self.get_logger().info(f'🔍 No-frontier recovery behavior: {behavior}')
        
        # Execute no-frontier recovery behaviors
        if behavior == 'force_frontier_search':
            self.recovery_force_frontier_search()
        elif behavior == 'coverage_check':
            self.recovery_coverage_check()
        elif behavior == 'backtrack_far':
            self.recovery_backtrack_far()
        elif behavior == 'systematic_search':
            self.recovery_systematic_search()
        elif behavior == 'spin_and_scan':
            self.recovery_spin_and_scan()
        elif behavior == 'random_walk':
            self.recovery_random_walk()
        elif behavior == 'emergency_stop':
            self.recovery_emergency_stop()

    def recovery_backtrack(self):
        """Backtrack to a previous known good position"""
        self.publish_status('backtracking_to_previous_position')
        
        if len(self.pose_history) < 8:
            self.get_logger().warn('⚠️ Insufficient pose history for backtracking, trying spin instead')
            self.recovery_spin_and_scan()
            return
        
        # Choose a position from earlier in the history (25% back)
        target_index = len(self.pose_history) // 4
        target_x, target_y, _ = self.pose_history[target_index]
        
        self.get_logger().info(f'🔙 Backtracking to position ({target_x:.2f}, {target_y:.2f})')
        
        # Send goal to navigation stack
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = target_x
        goal_msg.pose.position.y = target_y
        goal_msg.pose.orientation.w = 1.0  # Default orientation
        
        self.goal_pub.publish(goal_msg)
        
        # Schedule recovery completion check
        self.create_timer(10.0, self.check_recovery_success)

    def recovery_spin_and_scan(self):
        """Spin in place to scan for new paths"""
        self.publish_status('spinning_and_scanning')
        self.get_logger().info('🔄 Spinning in place to scan for new paths')
        
        # Stop current motion
        self.publish_stop_command()
        time.sleep(0.5)
        
        # Perform controlled spin
        twist = Twist()
        twist.angular.z = 0.8  # Moderate spin speed
        
        # Spin for 360 degrees (approximately)
        spin_duration = 2 * math.pi / abs(twist.angular.z)
        end_time = time.time() + spin_duration
        
        while time.time() < end_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop spinning
        self.publish_stop_command()
        
        # Wait a moment for sensors to process
        time.sleep(1.0)
        
        # Schedule recovery completion check
        self.create_timer(5.0, self.check_recovery_success)

    def recovery_random_walk(self):
        """Perform a short random walk to escape local minima"""
        self.publish_status('executing_random_walk')
        self.get_logger().info('🎲 Executing random walk to escape local minimum')
        
        if not self.current_pose:
            self.recovery_spin_and_scan()
            return
        
        # Generate a random nearby goal
        current_x = self.current_pose.pose.pose.position.x
        current_y = self.current_pose.pose.pose.position.y
        
        # Random offset (1-2 meters in random direction)
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(1.0, 2.5)
        
        target_x = current_x + distance * math.cos(angle)
        target_y = current_y + distance * math.sin(angle)
        
        self.get_logger().info(f'🎯 Random walk target: ({target_x:.2f}, {target_y:.2f})')
        
        # Send random goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = target_x
        goal_msg.pose.position.y = target_y
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)
        
        # Schedule recovery completion check
        self.create_timer(15.0, self.check_recovery_success)

    def recovery_skip_frontier(self):
        """Skip the current unreachable frontier"""
        self.publish_status('skipping_current_frontier')
        self.get_logger().info('⏭️ Skipping current unreachable frontier')
        
        skip_msg = String()
        skip_msg.data = "skip_current"
        self.skip_pub.publish(skip_msg)
        
        # Also try to resume exploration
        resume_msg = String()
        resume_msg.data = "resume"
        self.explore_resume_pub.publish(resume_msg)
        
        # Wait and then complete recovery
        self.create_timer(3.0, self.complete_recovery)

    def recovery_force_frontier_search(self):
        """Force the exploration algorithm to search for new frontiers"""
        self.publish_status('forcing_frontier_search')
        self.get_logger().info('🔍 Forcing new frontier search with relaxed parameters')
        
        # Send command to force frontier search
        search_msg = String()
        search_msg.data = "force_search"
        self.frontier_search_pub.publish(search_msg)
        
        # Also resume exploration
        resume_msg = String()
        resume_msg.data = "resume"
        self.explore_resume_pub.publish(resume_msg)
        
        # Wait longer for frontier detection
        self.create_timer(8.0, self.check_recovery_success)

    def recovery_backtrack_far(self):
        """Backtrack to a much earlier position for better coverage"""
        self.publish_status('far_backtracking')
        
        if len(self.exploration_positions) < 5:
            self.get_logger().warn('⚠️ Insufficient position history for far backtracking')
            self.recovery_systematic_search()
            return
        
        # Go back much further (to first 20% of exploration)
        target_index = max(0, len(self.exploration_positions) // 5)
        target_x, target_y, _ = self.exploration_positions[target_index]
        
        self.get_logger().info(f'🔙 Far backtracking to early position ({target_x:.2f}, {target_y:.2f})')
        
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = target_x
        goal_msg.pose.position.y = target_y
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)
        
        # Resume exploration after reaching position
        self.create_timer(15.0, self.resume_exploration_after_goal)

    def recovery_systematic_search(self):
        """Perform systematic search pattern to find missed areas"""
        self.publish_status('systematic_search_pattern')
        self.get_logger().info('🗂️ Executing systematic search pattern')
        
        if not self.current_pose or not self.map_data:
            self.recovery_random_walk()
            return
        
        # Calculate search positions in a grid pattern around current position
        current_x = self.current_pose.pose.pose.position.x
        current_y = self.current_pose.pose.pose.position.y
        
        # Create systematic search positions
        search_positions = []
        search_radius = 3.0
        grid_size = 1.5
        
        for dx in [-search_radius, 0, search_radius]:
            for dy in [-search_radius, 0, search_radius]:
                if dx == 0 and dy == 0:
                    continue
                search_positions.append((current_x + dx, current_y + dy))
        
        # Choose a search position we haven't been to recently
        for search_x, search_y in search_positions:
            # Check if we've been near this position recently
            too_close = False
            for exp_x, exp_y, _ in self.exploration_positions[-10:]:  # Last 10 positions
                if math.sqrt((search_x - exp_x)**2 + (search_y - exp_y)**2) < 1.0:
                    too_close = True
                    break
            
            if not too_close:
                self.get_logger().info(f'🎯 Systematic search target: ({search_x:.2f}, {search_y:.2f})')
                
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = 'map'
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                goal_msg.pose.position.x = search_x
                goal_msg.pose.position.y = search_y
                goal_msg.pose.orientation.w = 1.0
                
                self.goal_pub.publish(goal_msg)
                self.create_timer(20.0, self.resume_exploration_after_goal)
                return
        
        # If all systematic positions are too close, do random walk
        self.recovery_random_walk()

    def recovery_coverage_check(self):
        """Check specific areas that might have been missed"""
        self.publish_status('checking_coverage_gaps')
        self.get_logger().info('📋 Checking for coverage gaps')
        
        if not self.map_data or not self.current_pose:
            self.recovery_random_walk()
            return
        
        # Analyze map for potential unexplored areas
        # This is a simplified version - in practice, you'd do more sophisticated analysis
        current_x = self.current_pose.pose.pose.position.x
        current_y = self.current_pose.pose.pose.position.y
        
        # Look for areas we haven't visited in our exploration history
        coverage_targets = []
        search_range = 5.0
        
        for test_x in [current_x - search_range, current_x, current_x + search_range]:
            for test_y in [current_y - search_range, current_y, current_y + search_range]:
                # Check if we've been near this area
                visited = False
                for exp_x, exp_y, _ in self.exploration_positions:
                    if math.sqrt((test_x - exp_x)**2 + (test_y - exp_y)**2) < 1.5:
                        visited = True
                        break
                
                if not visited:
                    coverage_targets.append((test_x, test_y))
        
        if coverage_targets:
            # Go to the first unvisited area
            target_x, target_y = coverage_targets[0]
            self.get_logger().info(f'🎯 Coverage check target: ({target_x:.2f}, {target_y:.2f})')
            
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = target_x
            goal_msg.pose.position.y = target_y
            goal_msg.pose.orientation.w = 1.0
            
            self.goal_pub.publish(goal_msg)
            self.create_timer(18.0, self.resume_exploration_after_goal)
        else:
            self.get_logger().info('✅ No obvious coverage gaps found')
            self.recovery_random_walk()

    def resume_exploration_after_goal(self):
        """Resume exploration after reaching a recovery goal"""
        self.get_logger().info('🔄 Resuming exploration after recovery movement')
        
        # Force new frontier search
        search_msg = String()
        search_msg.data = "force_search"
        self.frontier_search_pub.publish(search_msg)
        
        # Resume exploration
        resume_msg = String()
        resume_msg.data = "resume"
        self.explore_resume_pub.publish(resume_msg)
        
        # Check if recovery was successful
        self.create_timer(10.0, self.check_no_frontier_recovery_success)

    def check_no_frontier_recovery_success(self):
        """Check if no-frontier recovery was successful"""
        if not self.is_recovering:
            return
        
        # If exploration has resumed (frontiers found), recovery was successful
        if not self.exploration_stopped:
            self.get_logger().info('✅ No-frontier recovery successful! Exploration resumed')
            self.complete_recovery()
            return
        
        # Otherwise, increment attempts and try next behavior
        self.recovery_attempts += 1
        self.get_logger().warn(f'⚠️ No-frontier recovery attempt {self.recovery_attempts} unsuccessful')
        self.execute_no_frontier_recovery()

    def recovery_emergency_stop(self):
        """Emergency stop and request manual intervention"""
        self.publish_status('emergency_stop_manual_required')
        self.get_logger().error('🛑 Emergency stop - manual intervention required')
        
        self.publish_stop_command()
        self.request_manual_intervention()

    def publish_stop_command(self):
        """Publish zero velocity to stop the robot"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def check_recovery_success(self):
        """Check if recovery was successful and complete the process"""
        if not self.is_recovering:
            return
            
        # Simple success check: if robot has moved since recovery started
        if len(self.pose_history) >= 2:
            recovery_start_pose = self.pose_history[0]
            current_pose = self.pose_history[-1]
            
            x0, y0, _ = recovery_start_pose
            x1, y1, _ = current_pose
            
            distance_since_recovery = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
            
            if distance_since_recovery > 0.5:  # Moved at least 0.5m
                self.get_logger().info(f'✅ Recovery successful! Moved {distance_since_recovery:.2f}m')
                self.complete_recovery()
                return
        
        # If recovery didn't work, try next behavior
        self.get_logger().warn('⚠️ Recovery attempt unsuccessful, trying next behavior')
        self.execute_recovery_behavior()

    def complete_recovery(self):
        """Mark recovery as complete and reset state"""
        self.is_recovering = False
        self.pose_history.clear()  # Clear history to get fresh start
        self.orientation_history.clear()
        
        self.get_logger().info('🎉 Recovery sequence completed successfully')
        self.publish_status('recovery_completed')
        
        # Publish recovery inactive status
        recovery_msg = Bool()
        recovery_msg.data = False
        self.recovery_active_pub.publish(recovery_msg)

    def request_manual_intervention(self):
        """Request manual intervention and stop autonomous operation"""
        self.publish_stop_command()
        self.recovery_attempts = self.max_recovery_attempts + 1  # Prevent further attempts
        self.is_recovering = False
        
        # Could integrate with fleet management system here
        self.get_logger().error('🆘 Manual intervention requested - autonomous recovery failed')

    def publish_status(self, status):
        """Publish current recovery status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        
    def reset_recovery_attempts(self):
        """Reset recovery attempts (can be called externally)"""
        self.recovery_attempts = 0
        self.get_logger().info('🔄 Recovery attempts counter reset')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ExplorationRecovery()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()