from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    robot_name = LaunchConfiguration('robot_name')
    robot_name_arg = DeclareLaunchArgument(
                    name='robot_name',
                    description='[ARG] name of the robot in Gazebo sim')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_sim_arg = DeclareLaunchArgument(
                    'use_sim_time',
                    default_value=use_sim_time,
                    description='If true, use simulated clock')
    world_name = LaunchConfiguration('world_name')
    world_name_arg = DeclareLaunchArgument(
        name='world_name',
        description='[ARG] name of the sdf file stored in the /world directory that is used to create the world for Gazbo sim',
        default_value='empty_world'
    )

    
    # find robot description pkg
    robot_pkg_path = FindPackageShare(
        PythonExpression(["'", robot_name, "_description", "'"])
    )

    # find robot world
    world_pkg_path = get_package_share_directory("robot_gz_startup")
    world_sdf_path = PathJoinSubstitution([
        world_pkg_path,
        "worlds",
        PythonExpression([
            "'", world_name, ".sdf", "'"
        ])
    ])

    ## sub function to lange state publisher
    def robot_state_publisher(context):
        robot_launch_path = PathJoinSubstitution([
            robot_pkg_path,
            "launch",
            PythonExpression([
                "'", robot_name, "_description.launch.py", "'"
            ])
        ])
        return [IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([robot_launch_path]),
                    launch_arguments={
                        'use_sim_time': use_sim_time
                    }.items())]
    
    # get robot controller config
    robot_controllers = PathJoinSubstitution([
        robot_pkg_path,
            "config",
            PythonExpression([
                "'", robot_name, "_controller.yaml", "'"
            ])
    ])

    env_var_resource = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([
            robot_pkg_path,
            'meshes'
        ])
    )
    
    env_var_plugin = SetEnvironmentVariable(
        'GZ_SIM_PLUGIN_PATH',
        ''
    )

    # spawn entity gazebo node
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'ackermann', '-allow_renaming', 'true', '-z', '0.1'],
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
                   '--param-file',
                   robot_controllers,
                   ],
    )
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )



    return LaunchDescription([
        bridge,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),
        OpaqueFunction(function=robot_state_publisher),
        gz_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner, LogInfo(msg='exit entity spawn')],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ackermann_steering_controller_spawner, LogInfo(msg='End joint_state_broadcaster_spawner')],
            )
        ),
        # Launch Arguments
        robot_name_arg,
        use_sim_arg,
        world_name_arg,
        # env vars
        env_var_plugin,
        env_var_resource,
    ])
