import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        prefix="xterm -e",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "stamped": True,
            },
        ],
        remappings=[('/cmd_vel', '/diff_cont/cmd_vel')],
    )

    # twist_stamper = Node(
    #     package='twist_stamper',
    #     executable='twist_stamper',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[
    #         ('/cmd_vel_in', '/diff_cont/cmd_vel_unstamped'),
    #         ('/cmd_vel_out', '/diff_cont/cmd_vel'),
    #         ],
    # )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),

        teleop_node,
        # twist_stamper,
    ])