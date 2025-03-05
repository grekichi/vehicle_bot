from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        parameters=[{'stamped': True}],
        remappings=[('cmd_vel', '/diff_cont/cmd_vel')]
    )

    return LaunchDescription([
        teleop_keyboard,
    ])