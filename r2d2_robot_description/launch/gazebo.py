import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import LaunchConfigurationEquals

import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the package paths for included packages
    pkg_r2d2_description = get_package_share_directory('r2d2_robot_description')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    # Get the model path and parse it
    robot_path = os.path.join(pkg_r2d2_description, 'urdf', 'r2d2.urdf.xacro')
    robot_parsed = xacro.process_file(robot_path)
    robot_description = {'robot_description': robot_parsed.toxml()}

    # Get the filepath for the ros2_controllers configuration file
    ros2_controllers_uri = os.path.join(pkg_r2d2_description, 'config/ros2_controllers.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        )
    )

    spawn_entity_args = [
        robot_description['robot_description'],
        LaunchConfiguration('spawn_x'),
        LaunchConfiguration('spawn_y'),
        LaunchConfiguration('spawn_z'),
        LaunchConfiguration('spawn_roll'),
        LaunchConfiguration('spawn_pitch'),
        LaunchConfiguration('spawn_yaw')
    ]

    spawn_entity = Node(
        package='r2d2_robot_description',
        executable='spawn_entity',
        output='screen',
        arguments=spawn_entity_args
    )

    # Load the robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='r2d2',
        output='screen',
        parameters=[robot_description],
    )

    # Load the controller_manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_uri],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    # Spawn the joint state broadcaster
    joint_state_broad = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '-c', '/r2d2/controller_manager', '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen'
    )

    # Spawn the head velocity controller
    head_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '-c', '/r2d2/controller_manager', '--set-state', 'start', 'head_velocity_controller'],
        output='screen'
    )

    # Spawn the head driver node
    # *allows easy access to ros2_control controller
    head_driver = Node(
        package='r2d2_robot_description',
        executable='head_driver',
        output='screen',
        arguments=['gazebo']
    )

    # Spawn the diff drive controller
    diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '-c', '/r2d2/controller_manager', '--set-state', 'start', 'diff_drive_controller'],
        output='screen'
    )

    movement_driver = Node(
        package='r2d2_robot_description',
        executable='movement_driver',
        output='screen',
        arguments=['gazebo']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'spawn_x',
            default_value=['0'],
            description='Spawn x corrdinate for the robot'
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value=['0'],
            description='Spawn y corrdinate for the robot'
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value=['0'],
            description='Spawn z corrdinate for the robot'
        ),
        DeclareLaunchArgument(
            'spawn_roll',
            default_value=['0'],
            description='Spawn roll corrdinate for the robot'
        ),
        DeclareLaunchArgument(
            'spawn_pitch',
            default_value=['0'],
            description='Spawn pitch corrdinate for the robot'
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value=['0'],
            description='Spawn yaw corrdinate for the robot'
        ),
        gazebo,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[robot_state_pub, joint_state_broad]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broad,
                on_exit=[head_velocity_controller, head_driver, diff_drive_controller, movement_driver]
            )
        )
    ])
