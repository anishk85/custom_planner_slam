#!/usr/bin/env python3
"""
Ultra-Aggressive Exploration Parameters for Complete House Coverage
With Advanced Recovery Behaviors and No-Frontier Handling
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Ultra-aggressive exploration node with complete coverage parameters
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            
            # Basic frame configuration
            {'robot_base_frame': 'base_link'},
            {'global_frame': 'map'},
            {'costmap_topic': 'global_costmap/costmap'},
            {'costmap_updates_topic': 'global_costmap/costmap_updates'},
            
            # Visualization
            {'visualize': True},
            {'publish_frontier_markers': True},
            
            # ULTRA-AGGRESSIVE FRONTIER DETECTION
            {'min_frontier_size': 0.02},  # Extremely small frontiers (2cm)
            {'frontier_search_radius': 15.0},  # Large search radius
            {'max_frontier_size': 50.0},  # Allow very large frontiers
            
            # AGGRESSIVE DISTANCE AND SAFETY
            {'min_distance': 0.005},  # Nearly zero minimum distance
            {'max_distance': 100.0},  # Very large maximum distance
            {'safety_radius': 0.1},  # Minimal safety radius
            
            # EXPLORATION BEHAVIOR - MAXIMUM AGGRESSION
            {'potential_scale': 0.05},  # Very low potential scale (less repulsion)
            {'gain_scale': 15.0},  # Very high gain scale (strong attraction to unexplored)
            {'orientation_scale': 0.0},  # No orientation preference
            {'distance_scale': 0.1},  # Low distance penalty
            
            # TIMING AND FREQUENCY - RAPID RESPONSE
            {'planner_frequency': 0.05},  # Very frequent planning (20Hz)
            {'progress_timeout': 600.0},  # Long timeout (10 minutes)
            {'potential_timeout': 120.0},  # Medium potential timeout
            {'transform_tolerance': 5.0},  # High transform tolerance
            
            # FRONTIER SELECTION STRATEGY
            {'frontier_travel_point': 'centroid'},  # Go to frontier center
            {'frontier_blacklist_timeout': 30.0},  # Short blacklist timeout
            
            # EXPLORATION PERMISSIONS - MAXIMUM FREEDOM
            {'allow_unknown': True},  # Explore unknown areas
            {'explore_clear_space': True},  # Explore all clear spaces
            {'explore_close_to_obstacles': True},  # Get close to obstacles
            
            # COVERAGE PARAMETERS - COMPLETE HOUSE EXPLORATION
            {'coverage_radius': 0.3},  # Small coverage radius for tight spaces
            {'exploration_radius': 0.2},  # Small exploration radius
            {'min_exploration_radius': 0.1},  # Minimum exploration radius
            
            # ADVANCED EXPLORATION SETTINGS
            {'use_frontier_big_density': True},  # Use dense frontier detection
            {'frontier_big_density_threshold': 0.8},  # High density threshold
            {'use_blacklist': True},  # Enable frontier blacklisting
            {'blacklist_radius': 0.2},  # Small blacklist radius
            
            # MOVEMENT AND PATH PLANNING
            {'move_base_timeout': 180.0},  # Long move base timeout
            {'unreachable_frontier_timeout': 60.0},  # Medium unreachable timeout
            {'stuck_timeout': 45.0},  # Stuck detection timeout
            
            # AGGRESSIVE RETRY SETTINGS
            {'retry_failed_frontiers': True},  # Always retry failed frontiers
            {'max_retry_attempts': 5},  # Multiple retry attempts
            {'retry_delay': 10.0},  # Short retry delay
            
            # EXPLORATION COMPLETION CRITERIA - VERY THOROUGH
            {'completion_percentage': 98.0},  # 98% completion required
            {'min_unexplored_area': 0.5},  # Very small minimum unexplored area
            {'completion_check_interval': 30.0},  # Check completion frequently
        ],
        remappings=[
            ('/move_base_simple/goal', '/move_base_simple/goal'),
            ('/move_base/result', '/move_base/result'),
            ('/move_base/status', '/move_base/status'),
        ]
    )

    # Enhanced recovery node with no-frontier handling
    recovery_node = Node(
        package='turtlebot3_autonomous_exploration',  # Your package name
        executable='exploration_recovery',
        name='exploration_recovery',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            
            # Recovery behavior parameters
            {'stuck_threshold_distance': 0.1},  # Very sensitive stuck detection
            {'stuck_threshold_time': 20.0},  # Quick stuck timeout
            {'recovery_cooldown': 30.0},  # Shorter cooldown
            {'max_recovery_attempts': 8},  # More recovery attempts
            
            # Coverage analysis parameters
            {'coverage_threshold': 0.95},  # 95% coverage threshold
            {'coverage_check_interval': 25.0},  # Frequent coverage checks
            
            # No-frontier recovery parameters
            {'no_frontier_timeout': 15.0},  # Quick no-frontier detection
            {'force_search_timeout': 30.0},  # Time to wait for forced search
            {'systematic_search_radius': 4.0},  # Radius for systematic search
            {'systematic_search_grid_size': 1.0},  # Grid size for systematic search
        ]
    )
    
    # Frontier monitor node (monitors exploration progress)
    frontier_monitor_node = Node(
        package='turtlebot3_autonomous_exploration',
        executable='frontier_monitor',
        name='frontier_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'monitor_frequency': 2.0},  # Check every 0.5 seconds
            {'no_frontier_threshold': 10.0},  # Trigger recovery after 10s with no frontiers
            {'coverage_report_interval': 60.0},  # Report coverage every minute
        ]
    )
    
    # Exploration supervisor node (ensures exploration doesn't stop prematurely)
    exploration_supervisor_node = Node(
        package='turtlebot3_autonomous_exploration',
        executable='exploration_supervisor',
        name='exploration_supervisor',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'supervision_frequency': 1.0},  # Check every second
            {'restart_exploration_delay': 5.0},  # Delay before restarting
            {'max_restart_attempts': 10},  # Maximum restart attempts
            {'exploration_timeout': 1800.0},  # 30 minute exploration timeout
        ]
    )

    # Delayed start for recovery systems (let exploration start first)
    delayed_recovery = TimerAction(
        period=10.0,  # Start recovery monitoring after 10 seconds
        actions=[recovery_node]
    )
    
    delayed_monitor = TimerAction(
        period=15.0,  # Start frontier monitoring after 15 seconds
        actions=[frontier_monitor_node]
    )
    
    delayed_supervisor = TimerAction(
        period=20.0,  # Start supervision after 20 seconds
        actions=[exploration_supervisor_node]
    )

    return LaunchDescription([
        use_sim_time_arg,
        explore_node,  # Start exploration immediately
        delayed_recovery,  # Recovery system
        delayed_monitor,  # Frontier monitoring
        delayed_supervisor,  # Exploration supervision
    ])