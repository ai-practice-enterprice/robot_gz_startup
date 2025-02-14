# robot_gz_startup
package to start the jetank or jetracer in Gazebo Sim in a world or the default empty world

While this package is used to launch the jetank and jetracer in particular for this project you can use this package to launch other robot packages. 

However some criteria must be met \ 
${ROBOTNAME} == your robot's name :
  1. the name of the robot package is:
     ```${ROBOTNAME}_description```
  1. the name of the robot's main xacro file is:
     ```${ROBOTNAME}_main.xacro```
  1. the the robot's main xacro file is located in a subdirectory called "urdf
     ```${ROBOTNAME}_description/urdf/${ROBOTNAME}_main.xacro```
  1. the name of the robot's launch file is:
     ```${ROBOTNAME}_description.launch.py```
  1. the the robot's launch file is located in a subdirectory called "launch"
     ```${ROBOTNAME}_description/launch/${ROBOTNAME}_description.launch.py```
  1. the name of the world inside this package is a ".sdf" file
     ```robot_gz_startup/world/my_world.sdf```
    
and therefore you're robot's package setup looks like this
```sh
# ros_ws/
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
 

If you wish to change these conventions you can change the "spawn_robot.launch.py" launch file, to your own discretion.  

![DDS Gazebo and ROS schema](.assest/DDS_and_ROS2_and_Gazebo_Sim.svg)


