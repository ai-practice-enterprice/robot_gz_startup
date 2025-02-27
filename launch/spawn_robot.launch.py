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
# - ExecuteProcess
#   -> evaluate and execute commands on runtime
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo , ExecuteProcess , RegisterEventHandler, OpaqueFunction

# - LaunchConfiguration
#   -> (enables to store a launch argument (argument is local and scoped to this file))
# - PythonExpression
#   -> (allows a mix of substitutions and variables of this script to be used together)
# - PathJoinSubstitution
#   -> (same "os.path.join" except is done async)
# - IfElseSubstitution
#   -> (returns 1 of 2 substitutions)
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution , IfElseSubstitution

# - PythonLaunchDescriptionSource
#   -> (tells ROS that the included file is Python based (others : XML or YAML))
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # (1) first we DEFINE the possible arguments (configurable via the CLI)
    # this tells ROS2 that a specific argument exists and may be provided
    # later on, to this script via the CLI or a other launch file/script
    robot_name = LaunchConfiguration('robot_name')
    world_name = LaunchConfiguration('world_name')
    gz_server_only = LaunchConfiguration('gz_server_only')

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
    gz_server_only_arg = DeclareLaunchArgument(
        name='gz_server_only',
        description='[ARG] choose whether you want to only launch the Gazebo server or both the GUI and server',
        default_value='false'
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
    
    # 5) the name of the world inside this package is ".sdf" file

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
    # 3) depending on 1) we can execute: 
    # 3.1) execute the gz_server.launch.py file 
    # or
    # 3.2) execute the gz_sim.launch.py file 
    #
    # both instances will be passed to the LaunchDescription but because of the IfCondition only 1 will execute
    
    # 1)
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    
    # 2)
    gz_launch_file = IfElseSubstitution(
        condition=gz_server_only,
        # 3.1)
        if_value=os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_server.launch.py'),
        # 3.2)
        else_value=os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'),
    )

    # check : https://gazebosim.org/docs/latest/ros2_launch_gazebo/
    # both launch files don't have the same argument names when loading the world which is
    # why the arguments must be handled dynamically
    world_arg_name = IfElseSubstitution(
        condition=gz_server_only,
        # If Server mode (gz_server), pass 'world_sdf_file' 
        if_value='world_sdf_file', 
        # If GUI mode    (gz_sim), pass 'gz_args'
        else_value='gz_args'            
    )

    world_arg = IfElseSubstitution(
        condition=gz_server_only,
        # If Server mode (gz_server), pass 'world.sdf' 
        if_value=world_sdf_path, 
        # If GUI mode    (gz_sim), pass '-r -v 4 world.sdf'
        else_value=PythonExpression(["'","-r -v 4 ",world_sdf_path,"'"])            
    )
    
    # 3)
    ros_gz_launch_desc = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(gz_launch_file),
        launch_arguments=[
            [ world_arg_name ,world_arg],
            ['on_exit_shutdown', 'true'],
        ],
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
    def create_bridge_node(context): # This delays the execution of that code until the launch context is available.
        robot_name_value = robot_name.perform(context)
        robot_package_name = f"{robot_name_value}_description"
        bridge_params = os.path.join(get_package_share_directory(robot_package_name),'config','gz_bridge.yaml')
        ros_gz_bridge_node = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}',
            ],
            output='screen'
        )
        return [ros_gz_bridge_node]
    
    opaque_ros_gz_bridge_node = OpaqueFunction(function=create_bridge_node)

    # while we could have put the "/camera/image_raw" in the parameter bridge, 
    # the image_bridge provides a more effecient bridge for image topics
    # see migration guide: https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/ 
    ros_gz_image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen'
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
            ['use_sim_time', 'true']
        ]
    )

    # 2)
    robot_controllers_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([robot_controllers_launch_path])
    )
    

    # (5) set the new environment variables
    # more on how and why to load these can be found here: https://gazebosim.org/api/sim/8/resources.html
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
        env_var_resource,
        # all the declared arguments are returned for if this launch file's arguments are not provided
        robot_name_arg,
        world_name_arg,
        gz_server_only_arg,
        # all the required Nodes + launch descriptions
        robot_launch_desc,
        ros_gz_launch_desc,
        opaque_ros_gz_bridge_node,
        ros_gz_image_bridge_node,
        spawn_entity_node,
        # all events
        launch_desc_after_entity_is_spawn
    ])
