# Copyright 2024 Your Name
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Test Launch File for Dijkstra Planner - Simplified Testing

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Test parameters with more verbose logging
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('dijkstra_planner'),
            'params',
            'dijkstra_test_params.yaml'))

    # Simple test map - create a basic map for testing
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to test param file'),

        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file'),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                        'yaml_filename': map_dir}]),

        # Lifecycle Manager for Map Server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': ['map_server']}]),

        # Planner Server (with our Dijkstra planner)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[param_dir, {'use_sim_time': use_sim_time}]),

        # Lifecycle Manager for Planner
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': ['planner_server']}]),

        # Static transform (for testing without robot)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']),

        # Print test instructions after delay
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['echo', 'Dijkstra Planner Test Setup Complete!'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['echo', 'Test planning with: ros2 service call /compute_path_to_pose nav2_msgs/srv/ComputePathToPose "{start: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 2.0}, orientation: {w: 1.0}}}}"'],
                    output='screen'
                )
            ]
        )
    ])