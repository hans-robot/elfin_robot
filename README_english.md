Elfin Robot
======

![elfin_robot](docs/images/elfin.png)

This repository provides ROS support for the Elfin Robot. The recommend operating environment is on Ubuntu 16.04 with ROS Kinetic or Ubuntu 14.04 with ROS Indigo. So far These packages haven't been tested in other environment.

### Installation

#### Ubuntu 16.04 + ROS Kinetic

**Install some important dependent software packages:**
```sh
$ sudo apt-get install ros-kinetic-soem ros-kinetic-gazebo-ros-control ros-kinetic-ros-control ros-kinetic-ros-controllers
```
**Install or upgrade MoveIt!.** 

If you have installed MoveIt!, please make sure that it's been upgraded to the latest version.

Install/Upgrade MoveIt!:

```sh
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-moveit
```

**Install this repository from Source**

First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like /path/to/your/catkin_workspace/src/cute_robot.  
Make sure to source the correct setup file according to your workspace hierarchy, then use catkin_make to compile.  

Assuming your catkin workspace folder is ~/catkin_ws, you should use the following commands:
```sh
$ cd ~/catkin_ws/src
$ git clone -b kinetic-devel https://github.com/hans-robot/elfin_robot.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

#### Ubuntu 14.04 + ROS Indigo

**Install some important dependent software packages:**
```sh
$ sudo apt-get install ros-indigo-soem ros-indigo-gazebo-ros-control ros-indigo-ros-control ros-indigo-ros-controllers
```
**Install or upgrade MoveIt!.** 

If you have installed MoveIt!, please make sure that it's been upgraded to the latest version.

Install/Upgrade MoveIt!:

```sh
$ sudo apt-get update
$ sudo apt-get install ros-indigo-moveit
$ sudo apt-get install ros-indigo-moveit-full-pr2
$ sudo apt-get install ros-indigo-moveit-kinematics
$ sudo apt-get install ros-indigo-moveit-ros-move-group
```

**Install this repository from Source**

First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like /path/to/your/catkin_workspace/src/cute_robot.  
Make sure to source the correct setup file according to your workspace hierarchy, then use catkin_make to compile.  

Assuming your catkin workspace folder is ~/catkin_ws, you should use the following commands:
```sh
$ cd ~/catkin_ws/src
$ git clone -b indigo-devel https://github.com/hans-robot/elfin_robot.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

---

### Usage with Gazebo Simulation

***There are launch files available to bringup a simulated robot - either Elfin3, Elfin5 or Elfin10.  
In the following the commands for Elfin3 are given. For Elfin5 or Elfin10, simply replace the prefix accordingly.***

Bring up the simulated robot in Gazebo:
```sh
$ roslaunch elfin_gazebo elfin3_empty_world.launch
```

Start up RViz with a configuration including the MoveIt! Motion Planning plugin:
```sh
$ roslaunch elfin3_moveit_config moveit_planning_execution.launch
```
If you don't want to start up RViz at the moment, just run:
```sh
$ roslaunch elfin3_moveit_config moveit_planning_execution.launch display:=false
```

Start up elfin basic api and "Elfin Control Panel" interface:
```sh
$ roslaunch elfin_basic_api elfin_basic_api.launch
```

> Tutorial about how to use MoveIt! RViz plugin: [docs/moveit_plugin_tutorial_english.md](docs/moveit_plugin_tutorial_english.md)  
Tips:
Every time you want to plan a trajectory, you should set the start state to current first.

For the teleoperation with joystick, run:
```sh
$ roslaunch elfin3_moveit_config joystick_control.launch
```
> Tutorial about joystick control teleoperation:  
http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/ros_visualization/joystick.html  
Tips:  
> 1. In the Motion Planning plugin of Rviz, enable “Allow External Comm.” checkbox in the “Planning” tab.  
> 2. Add “Pose” to rviz Displays and subscribe to /joy_pose in order to see the output from joystick. Note that only planning groups that have IK solvers for all their End Effector parent groups will work.

For more information about API, see [docs/API_description_english.md](docs/API_description_english.md)

---

###  Usage with real Hardware

***There are launch files available to bringup a real robot - either Elfin3, Elfin5 or Elfin10.  
In the following the commands for Elfin3 are given. For Elfin5 or Elfin10, simply replace the prefix accordingly.***

Put the file *elfin_drivers.yaml*, that you got from the vendor, into the folder elfin_robot_bringup/config/.

Connect Elfin to the computer with a LAN cable. Then confirm the ethernet interface name of the connection with `ifconfig`. The default ethernet name is eth0. If the ethernet name is not eth0, you should correct the following line in the file *elfin_robot_bringup/config/elfin_drivers.yaml* 

```
elfin_ethernet_name: eth0
```

Load Elfin robot model：
```sh
$ roslaunch elfin_robot_bringup elfin3_bringup.launch
```

Bring up the hardware of Elfin. Before bringing up the hardware, you should setup Linux with PREEMPT_RT properly. There is a [tutorial](https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/preemptrt_setup). There are two versions of elfin EtherCAT slaves. Please bring up the hardware accordingly.

```sh
$ sudo chrt 10 bash
$ roslaunch elfin_robot_bringup elfin_ros_control.launch
```
or
```sh
$ sudo chrt 10 bash
$ roslaunch elfin_robot_bringup elfin_ros_control_v2.launch
```

Start up RViz with a configuration including the MoveIt! Motion Planning plugin:
```sh
$ roslaunch elfin3_moveit_config moveit_planning_execution.launch
```
If you don't want to start up RViz at the moment, just run:
```sh
$ roslaunch elfin3_moveit_config moveit_planning_execution.launch display:=false
```

Start up elfin basic api and "Elfin Control Panel" interface:
```sh
$ roslaunch elfin_basic_api elfin_basic_api.launch
```

Enable the servos of Elfin with "Elfin Control Panel" interface: if there is no "Warning", just press the "Servo On" button to enable the robot. If there is "Warning", press the "Clear Fault" button first and then press the "Servo On" button.

> Tutorial about how to use MoveIt! RViz plugin: [docs/moveit_plugin_tutorial_english.md](docs/moveit_plugin_tutorial_english.md)  
Tips:
Every time you want to plan a trajectory, you should set the start state to current first.

For the teleoperation with joystick, run:
```sh
$ roslaunch elfin3_moveit_config joystick_control.launch
```
> Tutorial about joystick control teleoperation:  
http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/ros_visualization/joystick.html  
Tips:  
> 1. In the Motion Planning plugin of Rviz, enable “Allow External Comm.” checkbox in the “Planning” tab.  
> 2. Add “Pose” to rviz Displays and subscribe to /joy_pose in order to see the output from joystick. Note that only planning groups that have IK solvers for all their End Effector parent groups will work.

Before turning the robot off, you should press the "Servo Off" button to disable the robot.

For more information about API, see [docs/API_description_english.md](docs/API_description_english.md)
