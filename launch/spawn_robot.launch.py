# in this launch script we launch the Gazebo program
# together with the packages that are required to let ROS and Gazebo
# communicate with each other. These packages are already provided by the "ros_gz" package:
# - ros_gz_bridge   (enables translation between ROS and Gazebo Topics)
# - ros_gz_sim      (provides a launch script that can take in arguments to launch Gazebo)
# additionally the script takes in a few arguments (from the CLI or a other launch file/script)
# - robot package   (name of the package that containes already a robot's description +  "robot_state_publisher" node active )
# - robot_name      (name of that robot will have in Gazebo sim)
# - world_name      (sdf file that contains the world description)
# finally we must add a environment variables or add to it the location of our robot's meshes
# in order for Gazebo to be able to launch our mehses
# - GZ_SIM_RESOURCE_PATH    (name of the env variable for additional files such as meshes)
# - GZ_SIM_PLUGIN_PATH      (name of the env variable where plugins are located for the sensors)
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

# alternative substitution type for:
# "from ament_index_python import get_package_share_directory"
from launch_ros.substitutions import FindPackageShare

# - DeclareLaunchArgument
#   -> (allows to define arguments passed from the CLI or launch file/script)
# - IncludeLaunchDescription
#   -> (enables us to fetch a launch file from a other package and pass arguments to it)
# - SetEnvironmentVariable
#   -> (sets a new env variable)
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo

# - LaunchConfiguration
#   -> (enables to store a launch argument (argument is local and scoped to this file))
# - PythonExpression
#   -> (allows a mix of substitutions and variables of this script to be used together)
# - PathJoinSubstitution
#   -> (same "os.path.join" except is done async)
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

# - PythonLaunchDescriptionSource
#   -> (tells ROS that the included file is Python based (others : XML or YAML))
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # (1) first we DEFINE the possible arguments (configurable via the CLI)
    # this tells ROS2 that a specific argument exists and may be provided
    # later on, to this script via the CLI or a other launch file/script
    robot_name = LaunchConfiguration('robot_name')
    world_name = LaunchConfiguration('world_name')

    # (2) next we DECLARE the arguments of the launch file
    # that can be passed (or not) by the CLI or a other launch file/script.
    # additionally we can set a default value. If no default is set then
    # these arguments will send a error message when this script is run without these arguments
    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        description='[ARG] name of the robot in Gazebo sim'
    )
    world_name_arg = DeclareLaunchArgument(
        name='world_name',
        description='[ARG] name of the sdf file stored in the /world directory that is used to create the world for Gazbo sim',
        default_value='empty_world'
    )

    # NOTE :
    # "LaunchConfiguration" -> these are the REFERENCES to the command line arguments
    # "DeclareLaunchArgument" -> these are the arguments you want to accept from the CLI or a other launch script/file

    # CONVENTIONS
    # the following paragraph is very important for naming coventions and in case we want to add
    # new robot packages later on:

    # 1) the name of the robot package is:             ${ROBOTNAME}_description
    # 2) the name of the robot's main xacro file is:   ${ROBOTNAME}_main.xacro
    # 3) the name of the robot's launch file is :      ${ROBOTNAME}_description.launch.py
    # 4) the name of the world inside this package is ".sdf" file

    # (3) we check if the passed files and package names exist in the filesystem
    world_pkg_path = get_package_share_directory("robot_gz_startup")

    robot_pkg_path = FindPackageShare(
        # because the final result between "robot_name" and the extension is a string
        # we put it bewteen single quotes to make it 1 single string
        PythonExpression(["'", robot_name, "_description", "'"])
    )

    robot_urdf_path = PathJoinSubstitution([
        robot_pkg_path,
        "urdf",
        PythonExpression([
            "'", robot_name, "_main.xacro", "'"
        ])
    ])
    robot_launch_path = PathJoinSubstitution([
        robot_pkg_path,
        "launch",
        PythonExpression([
            "'", robot_name, "_description.launch.py", "'"
        ])
    ])
    world_sdf_path = PathJoinSubstitution([
        world_pkg_path,
        "worlds",
        PythonExpression([
            "'", world_name, ".sdf", "'"
        ])
    ])

    # IMPROVEMENTS ============================================================

    # NOTE: dont know how to do async conditional checking
    # I know i have to either use the "IfCondition" module or "IfElseSubstitution" module
    # to create async conditions but i am not sure

    # if not os.path.isfile(world_sdf_path):
    #  raise RuntimeError(f"[ERROR] can not find {world_sdf_path}")
    # if not os.path.isfile(robot_urdf_path):
    #  raise RuntimeError(f"[ERROR] can not find {robot_urdf_path}")
    # if not os.path.isfile(robot_launch_path):
    #  raise RuntimeError(f"[ERROR] can not find {robot_launch_path}")

    # IMPROVEMENTS ============================================================

    # (4) start all nodes and programs by using the preexisting packages:
    # - ros_gz_sim
    # (use this package's launch file to make it easier to start Gazebo with additional arguments + spawn our robot)
    # - spawn_entity
    # (a Python script provided by the ros_gz_sim package to spawn a robot in Gazebo)
    # - ros_gz_bridge
    # (use this package's launch file that can also take in arguments to translate ROS topics to Gazebo topics)
    # - robot_state_publisher
    # (present (normally) in the included robot package launch file (if not it should be created in the robot's package))

    # (4.1) ros_gz_sim
    # we need 2 things :
    # 1) the path to ros_gz_sim launch file
    # 2) use the path to find the launch file and pass the required arguments to launch Gazebo with the world
    # NOTE: the Gazebo migration guide (from classic to sim) also provides a way to
    # to launch the Gazebo server and Gazebo GUI seperately but this can be changed later on

    # 1)
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')

    # 2)
    ros_gz_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args':
                PythonExpression(["r'", "-r ", world_sdf_path, "'"]),    
            'on_exit_shutdown': 
                'true',
        }.items()
    )

    # (4.2) spawn_entity
    # we need 1 thing:
    # 1) start a node that reads from the topic "robot_description" send out by the robot_state_publisher node
    # and execute the create command

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
                '-topic', 'robot_description',
                '-entity', robot_name,
                # for some reason the jetracer and other models spawn under the ground_plane which is why we add 1 unit to the Z coord
                '-z' , '0.1'
                # additional arguments can be provided later on to fit out needs
                # ...
        ]
    )

    # (4.3) ros_gz_bridge
    # NOTE:  while the "bridge" provides a way to communicate ROS topics to Gazebo topics and vice versa
    # it still requires manual configuration but this can be done through a YAML file which can be created later
    # to suit our needs.

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # <topic>@<ROS2_msg_type>@<Gazebo_msg_type>
            # the ROS message type is followed by:
            # -> "@" is a bidirectional bridge.
            # -> "[" is a bridge from Gazebo to ROS.
            # -> "]" is a bridge from ROS to Gazebo.
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            # more ROS2 and Gazebo topics can be found at: https://docs.ros.org/en/jazzy/p/ros_gz_bridge/
            # the bridge itself can be configured later on using the command:
            # ros2 run ros_gz_bridge parameter_bridge
        ],
        output='screen'
    )

    # (4.4) robot_state_publisher
    # we need 1 thing:
    # 1) the robot's package launch file located inside the launch directory
    # and with a name (by convention) "${ROBOTNAME}_description.launch.py"
    # where ROBOTNAME is the name of the robot
    robot_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_launch_path]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # (5) set the new environment variables
    # more on how and why to load these can be found here: https://gazebosim.org/api/sim/8/resources.html
    # NOTE: i noticed that commenting these out you can still run Gazebo. 
    # I believe this is because Gazebo's  resolves the meshes path by using the attribute in the URDF file.
    # further investigation might be required if anything doesn't load correctly.
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


    # (6) finally return the launch description
    return LaunchDescription([
        # all the env variables
        env_var_plugin,
        env_var_resource,
        # all the declared arguments are returned for if this launch file's arguments are not provided
        robot_name_arg,
        world_name_arg,
        # all the required Nodes + launch descriptions
        robot_launch_desc,
        ros_gz_launch_desc,
        spawn_entity_node,
        ros_gz_bridge_node,
    ])
