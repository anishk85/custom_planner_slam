import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    robot_name = LaunchConfiguration('robot_name', default='turtlebot3')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'small_house.world'
    )

    # Path to TurtleBot3 model
    # model_path = os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'),
    #     'models', 'turtlebot3_waffle', 'model.sdf'
    # )

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3',
        description='Robot name for namespace'
    )

    # Launch Gazebo server with ROS plugins
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', world, '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Launch Gazebo client
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Robot state publisher with namespace
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': robot_name
        }.items()
    )

    # Direct spawn entity node with delay and namespace
    # spawn_entity_cmd = TimerAction(
    #     period=5.0,
    #     actions=[
    #         Node(
    #             package='gazebo_ros',
    #             executable='spawn_entity.py',
    #             arguments=[
    #                 '-entity', robot_name,
    #                 '-file', model_path,
    #                 '-x', x_pose,
    #                 '-y', y_pose,
    #                 '-z', '0.01',
    #                 '-robot_namespace', robot_name
    #             ],
    #             output='screen',
    #             parameters=[{'use_sim_time': use_sim_time}]
    #         )
    #     ]
    # )

    ld = LaunchDescription()
    
    # Add launch argument declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    
    # Add Gazebo server
    ld.add_action(gzserver_cmd)
    
    # Add client if not headless
    if not os.environ.get('GAZEBO_HEADLESS'):
        ld.add_action(gzclient_cmd)
    
    # Add robot state publisher
    ld.add_action(robot_state_publisher_cmd)
    
    # Add spawn command with delay
    # ld.add_action(spawn_entity_cmd)

    return ld