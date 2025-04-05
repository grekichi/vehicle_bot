import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
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

    """ if you don't use below 'rsp', you should use this program (Aleksandar Haber version) """
    # # this is a relative path to the xacro file defining the model
    # modelFileRelativePath = 'description/robot.urdf.xacro'
    # # this is the absolute path to the model
    # pathModelFile = os.path.join(
    #     get_package_share_directory(package_name),
    #     modelFileRelativePath
    #     )
    # # get the robot description from the xacro model file
    # robotDescription = xacro.process_file(pathModelFile).toxml()
    # # Robot State Publisher Node
    # nodeRobotStatePublisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    #     )

    # base launch program
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'false',
            }.items()
        )

    # joystick set
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'joystick.launch.py'),
                    ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'set_vel': '/cmd_vel',
                    }.items(),
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

    # setting for gz_args
    world_path = os.path.join(gz_model_path, sdf_file_name)
    gz_args = f'-r -v 4 {world_path}'

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gazebo_params.yaml'
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={
                        'gz_args': gz_args,
                        'on_exit_shutdown': 'true',
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                    }.items(),
        )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robotXacroName
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},]
    )
    
    # this is very important so we can control the robot from ROS2
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'parameters',
        'gz_bridge.yaml'
    )

    gz_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # ros_gz_image setting
    gz_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
            'camera.image.compressed.jpeg_quality': 75},
        ],
    )

    # Relay node to republish /camera/camera_info to /camera/image/camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )


    # addition of teleop twist keyboard 
    teleop_keyboard = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        prefix="xterm -e",
        parameters=[{'stamped': True}],
        # remappings=[('/cmd_vel', '/cmd_vel')]
        )

    # rviz2 load setting
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'actual_bot.rviz')
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Launch them all!
    return LaunchDescription([
        setLaunchConfig,
        setEnvVariable,
        rsp,
        # joystick,
        gazebo,
        spawnModelNodeGazebo,
        # nodeRobotStatePublisher,
        gz_ros_bridge_cmd,
        gz_ros_image_bridge_cmd, # addition
        relay_camera_info_node, # addition
        teleop_keyboard,
        rviz,
    ])
