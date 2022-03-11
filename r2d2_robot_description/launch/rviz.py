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
    model_uri = os.path.join(pkg_r2d2_robot_description, 'urdf', 'r2d2.urdf.xacro')
    model_description_config = xacro.process_file(model_uri)
    model_description = model_description_config.toxml()

    rviz_config_uri = os.path.join(pkg_r2d2_robot_description, 'config/r2d2.rviz')

    # Load the robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{"robot_description": model_description}],
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
        rviz
    ])




