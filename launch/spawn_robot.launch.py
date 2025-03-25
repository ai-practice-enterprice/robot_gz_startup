# in this launch script we launch the Gazebo program
# together with the packages that are required to let ROS and Gazebo
# communicate with each other. These packages are already provided by the "ros_gz" package:
# - ros_gz_bridge   (enables translation between ROS and Gazebo Topics)
# - ros_gz_sim      (provides a launch script that can take in arguments to launch Gazebo)
# - ros_gz_image    (provides a type of bridge for image transport effeciency between Gazebo and ROS)
# additionally the script takes in a few arguments (from the CLI or a other launch file/script)
# - robot_name      (name of that robot will have in Gazebo sim)
# - world_name      (sdf file that contains the world description)
# - gz_server_only  (boolean for the running only the server or both the GUI and server)
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
# - ExecuteProcess
#   -> evaluate and execute commands on runtime
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo , ExecuteProcess , RegisterEventHandler, GroupAction, AppendEnvironmentVariable

# - LaunchConfiguration
#   -> (enables to store a launch argument (argument is local and scoped to this file))
# - PythonExpression
#   -> (allows a mix of substitutions and variables of this script to be used together)
# - PathJoinSubstitution
#   -> (same "os.path.join" except is done async)
# - IfElseSubstitution # not in humble
#   -> (returns 1 of 2 substitutions)
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution , TextSubstitution

# - PythonLaunchDescriptionSource
#   -> (tells ROS that the included file is Python based (others : XML or YAML))
from launch.launch_description_sources import PythonLaunchDescriptionSource

# - OnProcessExit
#   -> (when running a EventHandler module (see RegisterEventHandler) you must tell it on WHICH event it has to execute what is inside the EventHandler OnProcessExit is one of these events)
from launch.event_handlers import OnProcessExit

# - IfCondition
#   -> (A condition to be checked at launchtime if the result == true then the node or action is executed)
# - UnlessCondition
#   -> (A condition to be checked at launchtime if the result == false then the node or action is executed)
from launch.conditions import IfCondition , UnlessCondition

def generate_launch_description():

    # (1) first we DEFINE the possible arguments (configurable via the CLI)
    # this tells ROS2 that a specific argument exists and may be provided
    # later on, to this script via the CLI or a other launch file/script
    robot_name = LaunchConfiguration('robot_name')
    world_name = LaunchConfiguration('world_name')
    gz_server_only = LaunchConfiguration('gz_server_only')
    robot_namespace = LaunchConfiguration('ns')
    # this is a important argument when running a simulation in Gazebo
    # setting this to 'true' will result in not launching certain nodes that are only 
    # required when first booting up the Gazebo server (and GUI)
    add_robot = LaunchConfiguration('add_robot')
    # spawning extra robots can should also include a x,y coordinate
    robot_x_coord = LaunchConfiguration('robot_x_coord')
    robot_y_coord = LaunchConfiguration('robot_y_coord')

    # (2) next we DECLARE the arguments of the launch file
    # that can be passed (or not) by the CLI or a other launch file/script.
    # additionally we can set a default value. If no default is set then
    # these arguments will send a error message when this script is run without these arguments
    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        description='[ARG] name of the robot in Gazebo sim'
    )
    robot_namespace_arg = DeclareLaunchArgument(
        name='ns',
        description='[ARG] required namespace to keep entity (simulation), nodes and topics separate when running multiple robots in the same simulation'
    )
    world_name_arg = DeclareLaunchArgument(
        name='world_name',
        description='[ARG] name of the sdf file stored in the /world directory that is used to create the world for Gazbo sim',
        default_value='empty_world'
    )
    gz_server_only_arg = DeclareLaunchArgument(
        name='gz_server_only',
        description='[ARG] choose whether you want to only launch the Gazebo server or both the GUI and server',
        default_value='false'
    )
    add_robot_arg = DeclareLaunchArgument(
        name='add_robot',
        description='[ARG] when already running a simulation start add this argument',
        default_value='false'
    )
    robot_x_coord_arg = DeclareLaunchArgument(
        name='robot_x_coord',
        description='[ARG] when spawning a robot you might want to spawn it in a different location then (0,0,0) so you can add this x value',
        default_value='0'
    )
    robot_y_coord_arg = DeclareLaunchArgument(
        name='robot_y_coord',
        description='[ARG] when spawning a robot you might want to spawn it in a different location then (0,0,0) so you can add this y value',
        default_value='0'
    )
    # NOTE :
    # "LaunchConfiguration" -> these are the REFERENCES to the command line arguments
    # "DeclareLaunchArgument" -> these are the arguments you want to accept from the CLI or a other launch script/file

    # CONVENTIONS ============================================================

    # the following paragraph is very important for naming coventions and in case we want to add
    # new robot packages later on:

    # 1) the name of the robot package is:             
    # ${ROBOTNAME}_description
    
    # 2) the name of the robot's main xacro file is:   
    # ${ROBOTNAME}_main.xacro
    
    # 3) the name of the robot's launch file starting the robot_state_publisher is :      
    # ${ROBOTNAME}_description.launch.py
    
    # 4) the name of the robot's launch file starting the controllers is :      
    # ${ROBOTNAME}_controllers.launch.py
    
    # 5) the name of the robot's config file for the ros_gz_bridge is :
    # gz_bridge.yaml

    # 6) the name of the world inside this package is ".sdf" file

    # CONVENTIONS ============================================================

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
    robot_controllers_launch_path = PathJoinSubstitution([
        robot_pkg_path,
        "launch",
        PythonExpression([
            "'", robot_name, "_controllers.launch.py", "'"
        ])
    ])
    robot_gz_bridge_path = PathJoinSubstitution([
        robot_pkg_path,
        "config",
        "gz_bridge.yaml"
    ])
    world_sdf_path = PathJoinSubstitution([
        world_pkg_path,
        "worlds",
        PythonExpression([
            "'", world_name, ".sdf", "'"
        ])
    ])
    gazebo_params_file_path = PathJoinSubstitution([
        world_pkg_path,
        "config",
        "gazebo_config.yaml"
    ]) 

    # IMPROVEMENTS ============================================================

    # NOTE: dont know how to do async conditional checking
    # I know i have to either use the "IfCondition" module or "IfElseSubstitution" module or perhaps a "opaque" function
    # we can also use "PythonExpression" module but this is less recommended especially if "IfElseSubstitution" is available
    # to create async conditions but i am not sure

    # IMPROVEMENTS ============================================================




    # (4) start all nodes and programs by using the preexisting packages:
    # - ros_gz_sim 
    # (use this package's launch files to make it easier to start Gazebo with additional arguments + spawn our robot)
    # - spawn_entity
    # (a Python script provided by the ros_gz_sim package to spawn a robot in Gazebo)
    # - ros_gz_bridge
    # (use this package's launch file that can also take in arguments to translate ROS topics to Gazebo topics)
    # - robot_state_publisher
    # (present (normally) in the included robot package launch file (if not it should be created in the robot's package))

    # (4.1) ros_gz_sim
    # we need 3 things :
    # 1) find the path to 'ros_gz_sim' launch file
    # 2) check the user's input for the 'gz_server_only' argument to see if we launch the GUI and the server or only the server 
    # 2.1) execute the gz_server.launch.py file 
    # or
    # 2.2) execute the gz_sim.launch.py file 
    # 3) depending on the 'add_robot' we can either ignore: 
    
    # 1)
    ros_gz_sim_pkg_path = get_package_share_directory('gazebo_ros')
    

    # 2) and 3) use Python conditionals to determine launch file and arguments
    gz_server = PythonExpression([" False if ","'",gz_server_only,"'","=='false' else True "])
    
    gz_launch_file = PythonExpression([
        "'" + os.path.join(ros_gz_sim_pkg_path, 'launch', 'gzserver.launch.py') + "'" ,
        " if ",
        gz_server,
        " else ",
        "'" + os.path.join(ros_gz_sim_pkg_path, 'launch', 'gazebo.launch.py') + "'"
    ])

    LogInfo(msg=gz_launch_file)

    world_arg_name = PythonExpression([
        "'world_sdf_file' if ",
        gz_server,
        " else 'extra_gazebo_args'"
    ])

    world_arg = PythonExpression([
        "'", world_sdf_path, "'",
        " if ",
        gz_server,
        " else " ,
        "'--ros_args --params-file ' + ",
        "'",gazebo_params_file_path,"'"
    ])

    # 3) Launch the description
    ros_gz_launch_desc = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(gz_launch_file),
        launch_arguments={
            'world': world_sdf_path,
            'verbose': 'true',
            'gui': PythonExpression(["'false' if ", gz_server, " else 'true'"]) #remove gui and only launch the server
        }.items(),
        condition=UnlessCondition(add_robot)
    )

    # (4.2) spawn_entity
    # we need 1 thing:
    # 1) start a node that reads from the topic "robot_description" send out by the robot_state_publisher node
    # and execute the create command

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
                '-topic', PythonExpression(["'",robot_namespace,'/robot_description',"'"]),
                '-entity', robot_namespace,
                # for some reason the jetracer and other models spawn under the ground_plane which is why we add 1 unit to the Z coord
                '-z' , '0.1',
                # for robot position on the x,y plane
                '-x',robot_x_coord,
                '-y',robot_y_coord,
                # additional arguments can be provided later on to fit out needs
                # ...
        ]
    )

   

    # (4.4) robot_state_publisher && robot_controllers
    # we need 2 things:
    # 1) the robot's package launch file that start the robot_state_publisher node 
    # located inside the launch directory and with a name (by convention) "${ROBOTNAME}_description.launch.py"
    # 2) the robot's package launch file that start the controllers nodes
    # located inside the launch directory and with a name (by convention) "${ROBOTNAME}_controllers.launch.py"
    # where ${ROBOTNAME} is the name of the robot

    # 1)
    robot_launch_desc = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([robot_launch_path]),
        launch_arguments=[
            ['use_sim_time', 'true'],
            ['ns',robot_namespace]
        ]
    )

    # 2)
    robot_controllers_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([robot_controllers_launch_path]),
        launch_arguments=[
            ['use_sim_time', 'true'],
            ['ns',robot_namespace],
        ]
    )
    

    # (5) set the new environment variables
    # more on how and why to load these can be found here: https://gazebosim.org/api/sim/8/resources.html
    # if we add a robot then we append the meshes to the env variable same goes for world meshes
    # env_var_resource_robots=AppendEnvironmentVariable(
    #     'GZ_SIM_RESOURCE_PATH',
    #     PathJoinSubstitution([
    #         robot_pkg_path,
    #         'meshes'
    #     ]),
    # )

	# we add the the path to the mesh directory wich contains the meshes used by the world sdf file
    env_var_resource_worlds=SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        os.path.join(
            world_pkg_path,
            'meshes'
        ),
    )
    env_var_resource_GZ=AppendEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        "/usr/share/gazebo-11"
    )

    # noticed that when only launching the server this might be required due to the 
    # launch script 'gz_server.launch.py' NOT setting this. while the launch script 'gz_sim.launch.py' does
    env_var_plugin=SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        '/opt/ros/humble/lib/',
    )
    
    # (6) set some event handlers to start everything in a more lineair 
    # fashion and have some control in the sequence how nodes are started 
    # once the robot has spawn inside Gazebo only then we call the launch description
    # of the robot's controllers
    launch_desc_after_entity_is_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[robot_controllers_launch],
        )
    )
    
    # (7) finally return the launch description
    return LaunchDescription([
        # all the env variables
        env_var_plugin,
        # env_var_resource_robots,
        env_var_resource_worlds,
        env_var_resource_GZ,
        # all the declared arguments are returned for if this launch file's arguments are not provided
        robot_name_arg,
        world_name_arg,
        gz_server_only_arg,
        robot_namespace_arg,
        add_robot_arg,
        robot_x_coord_arg,
        robot_y_coord_arg,
        # all the required Nodes + launch descriptions
        robot_launch_desc,
        ros_gz_launch_desc,
        spawn_entity_node,
        # all events
        launch_desc_after_entity_is_spawn,
        # debug info
        LogInfo(msg=gz_launch_file),
        LogInfo(msg=world_arg_name),
        LogInfo(msg=world_arg)
    ])
