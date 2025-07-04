# dijkstra_navigation.launch.py
# Updated launch file with better error handling

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    dijkstra_planner_dir = get_package_share_directory('dijkstra_planner')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    
    # Set default paths
    default_map_file = os.path.join(dijkstra_planner_dir, 'maps', 'map.yaml')
    default_params_file = os.path.join(dijkstra_planner_dir, 'params', 'nav2_params_dijkstra.yaml')
    
    # Check if files exist and provide fallbacks
    if not os.path.exists(default_params_file):
        # Fallback to nav2 default params
        default_params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
    
    lifecycle_nodes = ['map_server',
                      'amcl',
                      'controller_server',
                      'smoother_server', 
                      'planner_server',
                      'behavior_server',
                      'bt_navigator',
                      'waypoint_follower',
                      'velocity_smoother']

    # Map fully qualified names to relative ones so the lifecycle manager works
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own lifecycle manager with custom configuration
    load_composable_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_yaml_file,
                         'use_sim_time': use_sim_time,
                         'params_file': params_file,
                         'autostart': autostart,
                         'use_composition': use_composition,
                         'use_respawn': use_respawn}.items(),
    )

    # Set environment variable for TurtleBot3 model
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    return LaunchDescription([
        # Set env var to print messages immediately to stdout
        stdout_linebuf_envvar,
        
        # Declare the launch arguments
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'autostart', 
            default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Whether to use composed bringup'),

        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Whether to respawn if a node crashes'),

        # Include the actual bringup launch file
        load_composable_nodes,

        # Launch RViz2 with custom config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(dijkstra_planner_dir, 'rviz', 'dijkstra_rviz.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(PythonExpression(['True'])),  # Always launch RViz
            remappings=remappings),
    ])