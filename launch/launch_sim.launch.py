import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='vehicle_bot' #<--- CHANGE ME

    # this is a relative path to the xacro file defining the model
    modelFileRelativePath = 'description/robot_core.xacro'

    # this is the absolute path to the model
    pathModelFile = os.path.join(
        get_package_share_directory(package_name),
        modelFileRelativePath
        )

    # get the robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py')),
                    launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    """ Gazebo modify sentence """
    gz_model_path = os.path.join(get_package_share_directory(package_name), 'worlds')

    # if you create your original sdf, you should set this name
    sdf_file_name = 'vehicle_test.sdf'

    setLaunchConfig = SetLaunchConfiguration(
        name='sdf_file',
        value=[TextSubstitution(text=sdf_file_name)]
    )

    setEnvVariable = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path)



    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gazebo_params.yaml'
        )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
                    launch_arguments={
                        'gz_args': ' -r -v 4 vehicle_test.sdf',
                        'extra_gazebo_args': '--ros-args --params-file' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim',
                        executable='create',
                        arguments=['-topic', 'robot_description', '-name', 'my_bot'],
                        output='screen')
    

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

    # # additional portion
    # diff_drive_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=["diff_cont"],
    # )

    # joint_broad_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=["joint_broad"],
    # )

    # Launch them all!
    return LaunchDescription([
        setLaunchConfig,
        setEnvVariable,
        rsp,
        gazebo,
        spawn_entity,
        nodeRobotStatePublisher,
        start_gazebo_ros_bridge_cmd,
        # diff_drive_spawner,
        # joint_broad_spawner,
    ])
