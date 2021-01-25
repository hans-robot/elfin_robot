力矩模式硬件接口
====

使用硬件接口*hardware_interface::EffortJointInterface*的控制器可以对Elfin进行力矩模式控制。控制器*effort_controllers/JointTrajectoryController*是一个很好的例子，它的使用方法如下：

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
    +   type: effort_controllers/JointTrajectoryController
    +   joints:
    +      - elfin_joint1
    +      - elfin_joint2
    +      - elfin_joint3
    +      - elfin_joint4
    +      - elfin_joint5
    +      - elfin_joint6
    +   gains:
    +      elfin_joint1: {p: 2000, d: 10, i: 50, i_clamp: 10}
    +      elfin_joint2: {p: 2000, d: 10, i: 50, i_clamp: 10}
    +      elfin_joint3: {p: 2000, d: 10, i: 50, i_clamp: 10}
    +      elfin_joint4: {p: 2000, d: 10, i: 50, i_clamp: 10}
    +      elfin_joint5: {p: 2000, d: 10, i: 50, i_clamp: 10}
    +      elfin_joint6: {p: 2000, d: 10, i: 50, i_clamp: 10}
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

    gains: PID相关参数  
    
    velocity_ff: 此参数会与相应轴的目标速度相乘，以得到相应轴的与速度相关的前馈力矩。

    **请注意： 上述例子中的PID参数并不是最优参数，请合理设置PID参数，否则可能会造成危险情况**

2. 按[README.md](../README.md)的说明正常启动机械臂。

3. 启动后即可使用力矩环控制器，此时可使用rqt_reconfigure调整pid参数。

    ```sh
    rosrun rqt_reconfigure rqt_reconfigure
    ```

    ![pid_reconfigure](images/pid_reconfigure.png)