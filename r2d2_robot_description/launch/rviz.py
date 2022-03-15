# Launch file for starting the r2d2 model in rviz
# Written by Robert Forristall

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

# Main ros launch function
def generate_launch_description():

    # Get the relative path for this package
    pkg_r2d2_robot_description = get_package_share_directory('r2d2_robot_description')

    # Get and load the r2d2 robot description
    model_uri = os.path.join(pkg_r2d2_robot_description, 'urdf', 'r2d2_rviz.urdf.xacro')
    model_description_config = xacro.process_file(model_uri)
    model_description = model_description_config.toxml()

    # get the filepath for the rviz configuration file
    rviz_config_uri = os.path.join(pkg_r2d2_robot_description, 'config/r2d2.rviz')

    # Get the filepath for the ros2_controllers configuration file
    ros2_controllers_uri = os.path.join(pkg_r2d2_robot_description, 'config/rviz_controllers.yaml')

    # Load the robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{"robot_description": model_description}],
    )

    # Load the controller_manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': model_description}, ros2_controllers_uri],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    # Spawn the joint state broadcaster
    joint_state_broad = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawn the head velocity controller
    head_velocity_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['head_velocity_controller'],
        output='screen'
    )

    # Spawn the diff drive controller
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    # Load rviz
    rviz =  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_uri]
    )

    return LaunchDescription([
        robot_state_pub,
        controller_manager,
        joint_state_broad,
        head_velocity_controller,
        diff_drive_controller,
        rviz
    ])




