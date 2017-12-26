 API description
=====
### Subscribed Topics:

* **elfin_basic_api/joint_goal (sensor_msgs/JointState)**  
make the robot move to a position in joint space after planning a trajectory.  
example: function_pub_joints() in elfin_robot_bringup/script/cmd_pub.py

* **elfin_basic_api/cart_goal (geometry_msgs/PoseStamped)**  
make the robot move to a position in cartesian coordination system after planning a trajectory.  
example: function_pub_cart_xxx() in elfin_robot_bringup/script/cmd_pub.py

* **elfin_arm_controller/command (trajectory_msgs/JointTrajectory)**  
This topic contain a trajectory. When you publish this topic, the robot will move along the trajectory.

* **elfin_teleop_joint_cmd_no_limit (std_msgs/Int64)**  
This is a topic for developer, customers are not suggested to use it. A particular joint will move a little distance after subscribing this topic for once.

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

------
### Services:

* **elfin_basic_api/get_reference_link (std_srvs/SetBool)**  
You can get the reference link name of *MoveIt! Planning* from the response of this service.

* **elfin_basic_api/get_end_link (std_srvs/SetBool)**  
You can get the end link name of *MoveIt! Planning* from the response of this service.

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

***Following are the services, that support "Elfin Control Panel" interface.  customers are not suggested to use them.***

* **elfin_ros_control/elfin/enable_robot (std_srvs/SetBool)**  
Enable the robot. Please call elfin_basic_api/stop_teleop before calling this service. In this case the enabling process won't be disturbed by the motion commands, that were gaven before enabling.

* **elfin_ros_control/elfin/disable_robot (std_srvs/SetBool)**  
Disable the robot.  

* **elfin_ros_control/elfin/clear_fault (std_srvs/SetBool)**  
Clear fault.  

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
