# robot_gz_startup
package to start the jetank or jetracer in Gazebo Sim in a world or the default empty world. \
While this package is used to launch the jetank and jetracer in particular for this project you can use this package to launch other robot packages. \
However some criteria must be met in order for your robot package and this package to be used together.

**contents**
<ul>
  <li><a href="Requirements-robot-package">requirements: robot package</a></li>
  <li><a href="Setup-robot-package">setup: robot package</a></li>
  <li><a href="Gazebo Sim-,-ROS2-and-ros_gz_bridge">Gazebo Sim , ROS2 and ros_gz_bridge</a></li>
  <li><a href="How to use ?">How to use ?</a></li>
</ul>

## Requirements robot package 
`${ROBOTNAME}` == your robot's name \
`${WORLD}` == your world's name 
  1. the name of the robot package is\
     ```${ROBOTNAME}_description```
  1. the name of the robot's main xacro file is\
     ```${ROBOTNAME}_main.xacro```
  1. the the robot's main xacro file is located in a subdirectory called "urdf\
     ```${ROBOTNAME}_description/urdf/${ROBOTNAME}_main.xacro```
  1. the name of the robot's launch file is\
     ```${ROBOTNAME}_description.launch.py```
  1. the the robot's launch file is located in a subdirectory called "launch"\
     ```${ROBOTNAME}_description/launch/${ROBOTNAME}_description.launch.py```
  1. the name of the world inside this package is a ".sdf" file\
     ```robot_gz_startup/world/${WORLD}.sdf```

**If you wish to change these conventions you can change the "spawn_robot.launch.py" launch file, to your own discretion.**

## Setup robot package
If these criteria are followed you're robot package should like this
```sh
ros_ws/$ tree
# > ros_ws/
#    src/
#        ROBOTNAME_description/ [REQUIRED]
#              /launch [REQUIRED]
#                  /ROBOTNAME_description.launch.py [REQUIRED]
#              /urdf [REQUIRED]
#                  /ROBOTNAME_main.xacro [REQUIRED]
#                  /ROBOTNAME_links.xacro [OPTIONAL]
#                  /ROBOTNAME_joints.xacro [OPTIONAL]
#                  /ROBOTNAME_sensors.xacro [OPTIONAL]
#                  /... [OPTIONAL]
#              /meshes [OPTIONAL]
#                  /base_joint.sdl [OPTIONAL]
#                  /camera.sdl [OPTIONAL]
#                  /LiDAR.sdl [OPTIONAL]
#                  /wheels.sdl [OPTIONAL]
#                  /... [OPTIONAL]
```
## Gazebo Sim , ROS2 and ros_gz_bridge 
![DDS Gazebo and ROS schema](./assets/DDS_and_ROS2_and_Gazebo_Sim.svg)

## How to use ?

the `ros_gz` is shipped with some usefull packages such as:
  - ros_gz_bridge
  - ros_gz_sim

these 2 package are used in this package to make it easier to start gazebo with arguments \
and to translate ROS topics to Gazebo topics \
[➡️ more on these topics](https://docs.ros.org/en/jazzy/p/ros_gz_bridge/)

in order to use this package you place it in your ros workspace src/ directory \
and use colcon to rebuild your workspace. After you source your setup file and run the command

```sh
$ cd ~/ros_ws/src && git clone (this repo)
$ cd ~/ros_ws
# using the "symlink" flag will make sure that colcon uses symbolic links to your package
# and therefore does not copy directly all files which will be faster and enabled you to modify your files
# directy without rebuilding your workspace over and over  
$ colcon build --symlink-install
$ source install/setup.bash
```
afterwards you can use the package
```sh
# ${ROBOTNAME} == your robot name
# ${WORLD} == your world name
$ ros2 launch robot_gz_startup spawn_robot.launch.py robot_name:=${ROBOTNAME}
    # or
$ ros2 launch robot_gz_startup spawn_robot.launch.py robot_name:=${ROBOTNAME} world_name:=${WORLD}
```

## recommendations
i suggest that to further extend your project you add a .xacro file inside your robot package that is focused on 
the sensors of your robot. And further configure the ros_gz_bridge with a .YAML file and not change the launch file 
to keep a better overview in the launch file.



