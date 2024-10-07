import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    # this name has to match the robot name in the Xacro file
    robotXacroName='differential_drive_robot'

    # this is the name of our package, at the same time this is the name of the folder
    # that will be used to define the paths 
    package_name='vehicle_bot'

    # this is a relative path to the xacro file defining the model
    modelFileRelativePath = 'description/robot.urdf.xacro'

    # this is the absolute path to the model
    pathModelFile = os.path.join(
        get_package_share_directory(package_name),
        modelFileRelativePath
        )

    # get the robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # gazebo_params_file = os.path.join(
    #     get_package_share_directory(package_name),
    #     'config',
    #     'gazebo_params.yaml'
    #     )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazeboLaunch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
                    launch_arguments={
                        'gz_args': ['-r -v -v4 empty.sdf'],
                        'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', robotXacroName],
        output='screen'
    )
    
    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    # this is very important so we can control the robot from ROS2
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'parameters',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Launch them all!
    return LaunchDescription([
        gazeboLaunch,
        spawnModelNodeGazebo,
        nodeRobotStatePublisher,
        start_gazebo_ros_bridge_cmd,
    ])
