 # FinDER v3 Robot repository
 
 # Table of Contents
 * [Requirements](#requirements)
 * [Installation](#instalation)
 * [Usage](#usage)
  * [Simulation](#simulation)
    * [Simulation using Fake Node](#simulation-using-fake-node)
    * [Simulation using Gazebo](#simulation-using-gazebo)

# Requirements
- ROS
Installing dependencies:
`sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-
teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-
depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python
ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-
msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf
ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view
ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers`

`sudo apt-get install ros-melodic-cv-camera`

# Installation
## 1. Clone this repository
`git clone https://github.com/TRA-UNAM/FinderV3.git`

## 2. Install the eBUS_SDK
Change to the "ToInstall" directory:
`cd ~/FinderV3/ToInstall/`
Execute the file eBUS_SDK_4.1.7.3988_Ubuntu-14.04-x86_64.run:
`sudo ./eBUS_SDK_4.1.7.3988_Ubuntu-14.04-x86_64.run`

## 3. Compile the ROS workspace:
`cd ~/FinderV3/catkin_ws`
`catkin_make`

El último comando debe ejecutarse hasta llegar al 100% y sin mostrar mensajes de error (en letras
rojas).

**“source” del archivo “setup”**
Es necesario agregar cada nuevo workspace que se compile con catkin al entorno de ROS. Para ello,
correr el siguiente comando que hace “source” del nuevo archivo setup que se generó:
`source ~/FinderV3/catkin_ws/devel/setup.bash`

para no ejecutar este comando cada nueva terminal, es necesario agregar la siguiente línea al
archivo .bashrc:
`source ~/FinderV3/catkin_ws/devel/setup.bash`

# Usage
## Simulation
### Simulation using Fake Node

### Simulation using Gazebo
These instructions refer to run the simulation on gazebo **using the gazebo_ros package**, which is installed by default with every ROS distribution. More info here: http://gazebosim.org/tutorials?tut=ros_overview

#### 1. Simulate in empty world
Execute the command:
`roslaunch finder_gazebo finder_empty_world.launch`
to test the FinDERv3 robot in the gazebo default environment.
 
#### 2. Simulate in the Turtelbot3 maze
Execute the command:
`roslaunch finder_gazebo finder_stage_4.launch`
to test the FinDERv3 robot in the turtlebot3 maze.
 
#### 3. Simulate in a test world
Execute the command:
`roslaunch finder_gazebo finder_world_test.launch`
to test the FinDERv3 robot in a world with rough terrain.
 
#### Drive the FinDER v3 Robot
Execute the following command to teleoperate the robot with the keyboard:
`rosrun finder_teleop finder_teleop_key.py`
 
