 API description
=====
### Subscribed Topics:

* **elfin_basic_api/joint_goal (sensor_msgs/JointState)**  
make the robot move to a position in joint space after planning a trajectory.  
example: function_pub_joints() in elfin_robot_bringup/script/cmd_pub.py

* **elfin_basic_api/cart_goal (geometry_msgs/PoseStamped)**  
make the robot move to a position in cartesian coordination system after planning a trajectory.  
example: function_pub_cart_xxx() in elfin_robot_bringup/script/cmd_pub.py

* **elfin_basic_api/cart_path_goal (geometry_msgs/PoseArray)**  
make the robot move through the assigned positions in cartesian coordination system after planning a trajectory composed of lines.  
example: function_pub_cart_path_xxx() in elfin_robot_bringup/script/cmd_pub.py

* **elfin_arm_controller/command (trajectory_msgs/JointTrajectory)**  
This topic contain a trajectory. When you publish this topic, the robot will move along the trajectory.

* **elfin_teleop_joint_cmd_no_limit (std_msgs/Int64)**  
This is a topic for developer, customers are not supposed to use it. A particular joint will move a little distance after subscribing this topic for once.

	Meaning of the data in the topic:

	| data | joint       | direction |
	| ------- | ------------| -------------- |
	| 1 | elfin_joint1| ccw |
	| -1 | elfin_joint1 | cw |
	| 2 | elfin_joint2 | ccw |
	| -2 | elfin_joint2 | cw |
	| 3 | elfin_joint3| ccw |
	| -3 | elfin_joint3 | cw |
	| 4 | elfin_joint4 | ccw |
	| -4 | elfin_joint4 | cw |
	| 5 | elfin_joint5| ccw |
	| -5 | elfin_joint5 | cw |
	| 6 | elfin_joint6 | ccw |
	| -6 | elfin_joint6 | cw |

------
### Published Topics:

* **elfin_arm_controller/state (control_msgs/JointTrajectoryControllerState)**  
The current status of the joints.

* **elfin_ros_control/elfin/enable_state (std_msgs/Bool)**  
The servo status of the robot.  
true: enabled / false: disabled

* **elfin_ros_control/elfin/fault_state (std_msgs/Bool)**  
The fault status of the robot.  
true: warning / false: no fault

* **elfin_basic_api/parameter_updates (dynamic_reconfigure/Config)**  
The value of the dynamic parameters of elfin_basic_api, e.g. velocity scaling.

* **elfin_basic_api/reference_link_name (std_msgs/String)**  
The reference link in the calculations of the elfin_basic_api node

* **elfin_basic_api/end_link_name (std_msgs/String)**  
The end link in the calculations of the elfin_basic_api node

------
### Services:

* **elfin_basic_api/get_reference_link (std_srvs/SetBool)**  
You can get the reference link name of *elfin_basic_api* from the response of this service.

* **elfin_basic_api/get_end_link (std_srvs/SetBool)**  
You can get the end link name of *elfin_basic_api* from the response of this service.

* **elfin_basic_api/stop_teleop (std_srvs/SetBool)**  
Make the robot stop moving.

* **elfin_ros_control/elfin/get_txpdo (std_srvs/SetBool)**  
You can get the content of TxPDOs from the response of this service.

* **elfin_ros_control/elfin/get_rxpdo (std_srvs/SetBool)**  
You can get the content of RxPDOs from the response of this service.

* **elfin_ros_control/elfin/get_current_position (std_srvs/SetBool)**  
You can get the count values of the current joint positions from the response of this service.

* **elfin_basic_api/set_parameters (dynamic_reconfigure/Reconfigure)**  
Set the dynamic parameters of elfin_basic_api, e.g. velocity scaling  
example: set_parameters() in elfin_robot_bringup/script/set_velocity_scaling.py

* **elfin_ros_control/elfin/recognize_position (std_srvs/SetBool)**  
Recognize the position of joints.

* **elfin_ros_control/elfin/io_port1/write_do (elfin_robot_msgs/ElfinIODWrite)**  
Write a value into DO  
example:  
	```
	rosservice call /elfin_ros_control/elfin/io_port1/write_do "digital_output: 0x001b"
	```

* **elfin_ros_control/elfin/io_port1/read_di (elfin_robot_msgs/ElfinIODRead)**  
Read the value from DI  
example:  
	```
	rosservice call /elfin_ros_control/elfin/io_port1/read_di "data: true"
	```

* **elfin_ros_control/elfin/io_port1/get_txpdo (std_srvs/SetBool)**  
You can get the content of TxPDOs from the response of this service.

* **elfin_ros_control/elfin/io_port1/get_rxpdo (std_srvs/SetBool)**  
You can get the content of RxPDOs from the response of this service.

* **elfin_module_open_brake_slaveX(std_srvs/SetBool)**  
When the module is not enabled, you can open the brake of the corresponding module using this service.  
for example:
	```sh
	rosservice call elfin_module_open_brake_slave1 "data: true"
	```

* **elfin_module_close_brake_slaveX(std_srvs/SetBool)**  
When the module is not enabled, you can close the brake of the corresponding module using this service.  
for example:
	```sh
	rosservice call elfin_module_close_brake_slave1 "data: true"
	```

***Following are the services, that support "Elfin Control Panel" interface.  customers are not supposed to use them.***

* **elfin_basic_api/enable_robot (std_srvs/SetBool)**  
Enable the robot.  

* **elfin_ros_control/elfin/enable_robot (std_srvs/SetBool)**  
The recommand service for enabling the robot is *elfin_basic_api/enable_robot*. The robot will be enabled directly when you call this service. So you may need to deal with the status of controllers by yourself. For details: http://wiki.ros.org/controller_manager .

* **elfin_basic_api/disable_robot (std_srvs/SetBool)**  
Disable the robot.  

* **elfin_ros_control/elfin/disable_robot (std_srvs/SetBool)**  
The recommand service for disabling the robot is *elfin_basic_api/disable_robot*. The robot will be disabled directly when you call this service. So you may need to deal with the status of controllers by yourself. For details: http://wiki.ros.org/controller_manager .

* **elfin_ros_control/elfin/clear_fault (std_srvs/SetBool)**  
Clear fault.  

* **elfin_basic_api/set_reference_link (elfin_robot_msgs/SetString)**  
Set the reference link in the calculations of the elfin_basic_api node  

* **elfin_basic_api/set_end_link (elfin_robot_msgs/SetString)**  
Set the end link in the calculations of the elfin_basic_api node  

* **elfin_basic_api/joint_teleop (elfin_robot_msgs/SetInt16)**  
When this service is called, a particular joint will move in a direction and will NOT stop until it reach the limit position or elfin_basic_api/stop_teleop is called. Please be careful when you call this service.

	Meaning of the data in the service:

	| data | joint       | direction |
	| ------- | ------------| -------------- |
	| 1 | elfin_joint1| ccw |
	| -1 | elfin_joint1 | cw |
	| 2 | elfin_joint2 | ccw |
	| -2 | elfin_joint2 | cw |
	| 3 | elfin_joint3| ccw |
	| -3 | elfin_joint3 | cw |
	| 4 | elfin_joint4 | ccw |
	| -4 | elfin_joint4 | cw |
	| 5 | elfin_joint5| ccw |
	| -5 | elfin_joint5 | cw |
	| 6 | elfin_joint6 | ccw |
	| -6 | elfin_joint6 | cw |

* **elfin_basic_api/cart_teleop (elfin_robot_msgs/SetInt16)**  
When this service is called, the end link of the robot will move in a direction in cartsian coordination system and will NOT stop until it reach the limit position or elfin_basic_api/stop_teleop is called. Please be careful when you call this service.

	Meaning of the data in the service:

	| data | axis       | direction |
	| ------- | ------------| -------------- |
	| 1 | X | positive |
	| -1 | X | negative |
	| 2 | Y | positive |
	| -2 | Y | negative |
	| 3 | Z | positive |
	| -3 | Z | negative |
	| 4 | Rx | ccw |
	| -4 | Rx | cw |
	| 5 | Ry | ccw |
	| -5 | Ry | cw |
	| 6 | Rz | ccw |
	| -6 | Rz | cw |

* **elfin_basic_api/home_teleop (std_srvs/SetBool)**  
When this service is called, the robot will move to home position and will NOT stop until it reach the home position or elfin_basic_api/stop_teleop is called. Please be careful when you call this service.
