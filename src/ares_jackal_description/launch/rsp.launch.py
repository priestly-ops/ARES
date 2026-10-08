from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory('ares_jackal_description')

    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'jackal.urdf.xacro'
    )

    robot_description_config = xacro.process_file(xacro_file)

    robot_description = {
        'robot_description': robot_description_config.toxml()
    }

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                {'use_sim_time': True}
            ]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
    ])