Velocity + Torque feed forward mode hardware interface
====

A controller with the hardware interface *'elfin_hardware_interface::PosVelTrqJointInterface'* can control the Elfin in velocity + torque feed forward mode. **Only a robot with the version 2 of elfin EtherCAT slaves supports this hardware interface**.  
There is a sample example in the *elfin_ros_controllers* package: *'elfin_pos_vel_controllers/JointTrajectoryController'*. You can use it by the following method.

1. Change the controller type in the file *elfin_robot_bringup/config/elfin_arm_control.yaml*:

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
    
    feedforward velocity: desired velocity  
    feedforward torque: 0

2. Start the robot arm normally as described in the [README_english.md](../README_english.md)