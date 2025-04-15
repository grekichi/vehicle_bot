import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    set_vel = LaunchConfiguration('set_vel')

    joy_params = os.path.join(
        get_package_share_directory('vehicle_bot'),
        'config',
        'joystick_ps.yaml',
        )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[
            joy_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[
            joy_params,
            {
                'use_sim_time': use_sim_time,
                'publish_stamped_twist': True,
            },
        ],
        # remappings=[('/cmd_vel', set_vel)],
        remappings=[('/cmd_vel', '/cmd_vel_joy')],

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

        # DeclareLaunchArgument(
        #     'set_vel',
        #     default_value='/diff_cont/cmd_vel',
        #     description='set vel for each launch method'
        # ),

        joy_node,
        teleop_node,
        # twist_stamper,
    ])