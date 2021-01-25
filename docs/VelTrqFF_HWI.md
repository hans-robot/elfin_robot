速度+力矩前馈模式硬件接口
====

使用硬件接口*elfin_hardware_interface::PosVelTrqJointInterface*的控制器可以对Elfin进行速度+力矩前馈模式控制。**只有使用Version 2版本EtherCAT从站的Elfin有这个接口**。  
*elfin_ros_controllers*软件包提供了一个简单的例子： *elfin_pos_vel_controllers/JointTrajectoryController*，它的使用方法如下：

1. 更改elfin_robot_bringup/config/elfin_arm_control.yaml中的控制器类型:

    ```diff
    - elfin_arm_controller:
    -   type: position_controllers/JointTrajectoryController
    -   joints:
    -      - elfin_joint1
    -      - elfin_joint2
    -      - elfin_joint3
    -      - elfin_joint4
    -      - elfin_joint5
    -      - elfin_joint6
    -   constraints:
    -       goal_time: 0.6
    -       stopped_velocity_tolerance: 0.1
    -   stop_trajectory_duration: 0.05
    -   state_publish_rate:  25
    -   action_monitor_rate: 10
    + elfin_arm_controller:
    +   type: elfin_pos_vel_controllers/JointTrajectoryController
    +   joints:
    +      - elfin_joint1
    +      - elfin_joint2
    +      - elfin_joint3
    +      - elfin_joint4
    +      - elfin_joint5
    +      - elfin_joint6
    +   constraints:
    +       goal_time: 0.6
    +       stopped_velocity_tolerance: 0.1
    +   stop_trajectory_duration: 0.05
    +   state_publish_rate:  25
    +   action_monitor_rate: 10

    ```
    
    前馈速度： 目标速度  
    前馈力矩： 0

2. 按[README.md](../README.md)的说明正常启动机械臂。