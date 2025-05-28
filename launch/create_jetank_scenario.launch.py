import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution , TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription , OpaqueFunction , LogInfo
from launch import LaunchDescription, LaunchContext
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def include_swarm_launch_descriptions(context: LaunchContext):
    robot_gz_startup_path = get_package_share_directory("robot_gz_startup")
    robot_gz_startup_launch_path = PathJoinSubstitution([
        robot_gz_startup_path,
        "launch",
        PythonExpression([
            "'","spawn_robot.launch.py", "'"
        ])
    ])

    actions = []
    num_robots_config = LaunchConfiguration("num_robots")
    world_name_config = LaunchConfiguration('world_name')

    num_robots = int(num_robots_config.perform(context))

    message = LogInfo(msg=f'Number of robots {num_robots}')

    actions.append(IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([robot_gz_startup_launch_path]),
            launch_arguments=[
                ('robot_name', 'jetank'),
                ('ns','jetank_1'),
                ('world_name',world_name_config.perform(context))
            ]
        )
    )

    actions.append(Node(
        package="jetank_navigation",
        executable="navigate",
        namespace='jetank_1',
    ))

    actions.append(Node(
        package="midas_node",
        executable="depthV2",
        namespace='jetank_1',
    )) 

    for n in range(1,num_robots):
        actions.append(Node(
            package="jetank_navigation",
            executable="navigate",
            namespace='jetank_' + str(n + 1),
        ))

        actions.append(Node(
            package="midas_node",
            executable="depthV2",
            namespace='jetank_' + str(n + 1),
        )) 

        m = n % 5

        actions.append(IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([robot_gz_startup_launch_path]),
            launch_arguments=[
                ['robot_name', 'jetank'],
                ['ns','jetank_' + str(n + 1)],
                ['add_robot','true'],
                ['robot_y_coord',str(m)],
                ['robot_z_coord',str(0.1)],
            ]
        )) 

    return actions + [message]


def generate_launch_description():

    num_robots = LaunchConfiguration("num_robots")
    world_name = LaunchConfiguration('world_name')

    num_robots_arg = DeclareLaunchArgument(
        name="num_robots",
        description="[ARG] number of robots that need to be in the simulation",
        default_value='2'
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        description="[ARG] world to use written without the .sdf extension",
        default_value='empty_world'
    )

    robot_gz_startup_path = get_package_share_directory("robot_gz_startup")
    robot_gz_startup_launch_path = PathJoinSubstitution([
        robot_gz_startup_path,
        "launch",
        PythonExpression([
            "'","spawn_robot.launch.py", "'"
        ])
    ])
    
    server_node = Node(
        package="jetank_navigation",
        executable="server",
    )

    return LaunchDescription([
        world_name_arg,
        num_robots_arg,
        OpaqueFunction(function=include_swarm_launch_descriptions),
        server_node
    ])