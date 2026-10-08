from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'ares_localization'
    )

    config_file = os.path.join(
        pkg_share,
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',

            parameters=[
                config_file,
                {'use_sim_time': True}
            ],

            remappings=[
                ('odometry/filtered',
                 '/odometry/filtered')
            ]
        )

    ])
