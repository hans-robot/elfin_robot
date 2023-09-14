Elfin Robot
======


Chinese version of the README -> please [click here](./README_cn.md)


<p align="center">
  <img src="docs/images/elfin.png" />
</p>

This repository provides ROS support for the Elfin Robot. The recommend operating environment is on Ubuntu 20.04 with ROS noetic. So far These packages haven't been tested in other environment.

### Installation

#### Ubuntu 20.04 + ROS noetic

**Install some important dependent software packages:**
```sh
$ sudo apt-get install ros-noetic-soem ros-noetic-gazebo-ros-control ros-noetic-ros-control ros-noetic-ros-controllers
```
**Install or upgrade MoveIt!.** 

If you have installed MoveIt!, please make sure that it's been upgraded to the latest version.

Install/Upgrade MoveIt!:

```sh
$ sudo apt-get update
$ sudo apt-get install ros-noetic-moveit-*
```

install trac_ik plugin
```sh
sudo apt-get install ros-noetic-trac-ik
```

**Related environment configuration**
Please ensure that the current system Python version is 3.8. 
If the system does not specify a python version after installation, please execute the following instructions

Check Python version
```sh
$ python --version
```

If the python version is not output, that is, the system does not specify a python version, execute the following instructions
Check the python version installed on the current system
```sh
$ ls /usr/bin/python*
```

Check Python configuration scheme
```sh
$ sudo update-alternativers --list python
```

Configure Python scheme
```sh
$ sudo update-alternatives --install /usr/bin/python python /usr/bin/python2 1
$ sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 2
```

Change Python Version
```sh
$ sudo update-alternatives --config python
```
Enter 2 to switch to python3

Set Python links
```sh
$ sudo ln -s /usr/bin/python3.8 python
```

If there are hava links, delete them first
```sh
$ sudo rm python
```
And set Python links again

After setting, please check the python version again to ensure that the version is 3.8
```sh
$ python --version
```

Install related software packages
```sh
$ sudo apt-get install build-essential libgtk-3-dev
$ sudo pip install wxpython
```

**Install this repository from Source**

First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like /path/to/your/catkin_workspace/src/elfin_robot.  
Make sure to source the correct setup file according to your workspace hierarchy, then use catkin_make to compile.  

Assuming your catkin workspace folder is ~/catkin_ws, you should use the following commands:
```sh
$ cd ~/catkin_ws/src
$ git clone -b noetic https://github.com/hans-robot/elfin_robot.git
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

Bring up the hardware of Elfin. Before bringing up the hardware, you should setup Linux with PREEMPT_RT properly. There is a [tutorial](https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/preemptrt_setup). There are two versions of elfin EtherCAT slaves. Please bring up the hardware accordingly.**The current branch is applicable to models with 485 end shapes. Please refer to the last column of this article to distinguish between bus end and 485 end.**

```sh
$ sudo chrt 10 bash
$ roslaunch elfin_robot_bringup elfin_ros_control.launch
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

Tutorial about how to use MoveIt! RViz plugin: [docs/moveit_plugin_tutorial_english.md](docs/moveit_plugin_tutorial_english.md)  
Tips:
Every time you want to plan a trajectory, you should set the start state to current first.

Before turning the robot off, you should press the "Servo Off" button to disable the robot.

For more information about API, see [docs/API_description_english.md](docs/API_description_english.md)

---
**Distinguish between bus end shape and 485 end shape, please refer to the difference in the sixth axis**

<p align="center">
  <img src="docs/images/485_END.png" />
  <br>
  485 END
</p>

---

<p align="center">
  <img src="docs/images/ethercat_END.png" />
  <br>
  Ethercat END
</p>