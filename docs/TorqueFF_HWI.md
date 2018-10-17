力矩前馈模式硬件接口
====

使用硬件接口*elfin_hardware_interface::PosTrqJointInterface*的控制器可以对Elfin进行力矩前馈模式控制。*elfin_ros_controllers*软件包提供了一个简单的例子： *elfin_pos_trq_controllers/JointTrajectoryController*，它的使用方法如下：

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
    +   type: elfin_pos_trq_controllers/JointTrajectoryController
    +   joints:
    +      - elfin_joint1
    +      - elfin_joint2
    +      - elfin_joint3
    +      - elfin_joint4
    +      - elfin_joint5
    +      - elfin_joint6
    +   velocity_ff:
    +      elfin_joint1: 1
    +      elfin_joint2: 1
    +      elfin_joint3: 1
    +      elfin_joint4: 1
    +      elfin_joint5: 1
    +      elfin_joint6: 1
    +   constraints:
    +       goal_time: 0.6
    +       stopped_velocity_tolerance: 0.1
    +   stop_trajectory_duration: 0.05
    +   state_publish_rate:  25
    +   action_monitor_rate: 10

    ```
    
    velocity_ff: 此参数会与相应轴的目标速度相乘，以得到相应轴的与速度相关的前馈力矩。

2. 按[README.md](../README.md)的说明正常启动机械臂。