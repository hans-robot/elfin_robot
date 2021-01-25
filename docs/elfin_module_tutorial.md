Elfin module tutorial
======

### 使用仿真模型

用Gazebo仿真请运行：
```sh
$ roslaunch elfin_gazebo elfin_module_empty_world.launch model:=module_xx # e.g. module_14
```
利用"elfin_module_controller/follow_joint_trajectory" action 可控制轴转动。elfin_robot_bringup/script/elfin_module_cmd_pub.py 是一个例程，运行它可让轴转动一定角度。
```sh
$ rosrun elfin_robot_bringup elfin_module_cmd_pub.py
```

---

### 使用真实的Elfin模组

先把购买模组时得到的elfin_drivers.yaml放到elfin_robot_bringup/config/文件夹下。

将模组通过网线连接到电脑。先通过`ifconfig`指令来确定与模组连接的网卡名称。本软件包默认的名称是eth0 。假如当前名称不是eth0的话，请对elfin_robot_bringup/config/elfin_drivers.yaml的相应部分进行修改。
```
elfin_ethernet_name: eth0
```

加载Elfin模组模型：
```sh
$ roslaunch elfin_robot_bringup elfin_module_bringup.launch model:=module_xx # e.g. module_14
```
启动硬件，Elfin的控制需要操作系统的实时性支持，运行下面的命令前请先为你的Linux系统内核打好实时补丁。打补丁的方法可以参考这个[教程](http://www.jianshu.com/p/8787e45a9e01)。Elfin机械臂有两种不同版本的EtherCAT从站，在启动硬件前，请先确认你的Elfin的从站版本。
```sh
$ sudo chrt 10 bash
$ roslaunch elfin_robot_bringup elfin_module_ros_control.launch 
```
或
```sh
$ sudo chrt 10 bash
$ roslaunch elfin_robot_bringup elfin_module_ros_control_v2.launch 
```
借助以下消息查看模组状态：  

* **elfin_ros_control/elfin/enable_state (std_msgs/Bool)**  
反映此时电机的使能状态。  
true: 使能  / false: 未使能

* **elfin_ros_control/elfin/fault_state (std_msgs/Bool)**  
反映此时电机的报错状态。  
true: 有报错  / false: 无报错

清错：
```sh
$ rosservice call /elfin_ros_control/elfin/clear_fault "data: true"
```

使能：  
```sh
$ rosservice call /elfin_ros_control/elfin/enable_robot "data: true"
```

启动控制器：  
如果你没有安装过rqt_controller_manager，请先安装它：
```sh
$ sudo apt-get install ros-<distro>-rqt-controller-manager
```
请用你的ROS版本名代替`<distro>`，例如：indigo, kinetic 。

使用rqt_controller_manager启动控制器
```sh
$ rosrun rqt_controller_manager rqt_controller_manager
```
![start_module_controller](images/start_module_controller.png)

控制模组：  
利用"elfin_module_controller/follow_joint_trajectory" action 可控制轴转动。elfin_robot_bringup/script/elfin_module_cmd_pub.py 是一个例程，运行它可让轴转动一定角度。
```sh
$ rosrun rosrun elfin_robot_bringup elfin_module_cmd_pub.py
```

去使能：
```sh
$ rosservice call /elfin_ros_control/elfin/disable_robot "data: true"
```

关闭控制器：  
使用rqt_controller_manager关闭控制器
```sh
$ rosrun rqt_controller_manager rqt_controller_manager
```
![stop_module_controller](images/stop_module_controller.png)
