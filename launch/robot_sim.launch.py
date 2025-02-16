import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    robot_name = LaunchConfiguration('robot_name')
    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        description='[ARG] name of the robot in Gazebo sim'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    def robot_state_publisher(context):
        # find robot description pkg
        robot_pkg_path = FindPackageShare(
            # because the final result between "robot_name" and the extension is a string
            # we put it bewteen single quotes to make it 1 single string
            PythonExpression(["'", robot_name, "_description", "'"])
        )
        robot_launch_path = PathJoinSubstitution([
            robot_pkg_path,
            "launch",
            PythonExpression([
                "'", robot_name, "_description.launch.py", "'"
            ])
        ])