import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    # this name has to match the robot name in the Xacro file
    robotXacroName='differential_drive_robot'

    package_name='vehicle_bot' #<--- If folder name changed, you should change here

    # robot state publisher node 
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py')
                    ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'use_ros2_control': 'true',
                    }.items(),
        )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py')
                    ]),
                launch_arguments={
                    'set_vel' : '/diff_cont/cmd_vel',
                    }.items(),
        )

    """ Gazebo modify sentence """
    gz_model_path = os.path.join(get_package_share_directory(package_name), 'worlds')

    # if you create your original sdf, you should set this name
    sdf_file_name = 'tennis_ball.sdf'  # 'vehicle_test.sdf'

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
                        get_package_share_directory('ros_gz_sim'),
                        'launch',
                        'gz_sim.launch.py')
                        ]),
                launch_arguments={
                    'gz_args': gz_args,
                    'on_exit_shutdown': 'true',
                    'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                    }.items()
             )
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robotXacroName,
            ],
        output='screen',
    )
    
    # this is very important so we can control the robot from ROS2
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'parameters',
        'gz_bridge_4sim.yaml',
    )

    gz_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
            ],
        )

    # ros_gz_image setting
    gz_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        output='screen',
        arguments=['/camera/image_raw'],
        parameters=[
            {
                'use_sim_time': True,
                'camera.image.compressed.jpeg_quality': 75
            },
        ]
        )

    # Relay node to republish /camera/camera_info to /camera/image_raw/camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['/camera/camera_info', '/camera/image_raw/camera_info'],
        parameters=[{'use_sim_time': True}],
        )

    # twist_mux section
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
        )
    
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[
                twist_mux_params,
                {
                    'use_sim_time': True,
                    'use_stamped': True,
                },
                ],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel')]
        )

    # additional portion
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["diff_cont"],
        )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_broad"],
        )

    # addition of teleop twist keyboard 
    teleop_keyboard = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        prefix="xterm -e",
        parameters=[{'stamped': True}],
        # remappings=[('/cmd_vel', '/diff_cont/cmd_vel')],
        remappings=[('/cmd_vel', '/cmd_vel_key')],
    )



    # Launch them all!
    return LaunchDescription([
        setLaunchConfig,
        setEnvVariable,
        rsp,
        # joystick,
        gazebo,
        spawn_entity,
        gz_ros_bridge_cmd,
        gz_ros_image_bridge_cmd,
        relay_camera_info_node,
        diff_drive_spawner,
        joint_broad_spawner,
        teleop_keyboard,
        twist_mux,
    ])
