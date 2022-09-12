elfin_robot_servo
=====
### start servo:

* **roslaunch elfin_robot_servo start_moveit_servo.launch**  

### Subscribed Topics 
* **/servo_server/delta_twist_cmds**  
令机械臂末端往x/y/z方向按照一定速度持续运动  
example: servo_movePose() in elfin_robot_servo/src/test.py

* **/servo_server/delta_joint_cmds**  
令机械臂各个关节往正/负运动方向按照一定速度持续运动  
example: servo_moveJoint() in elfin_robot_servo/src/test.py


### 参考:
```sh
https://ros-planning.github.io/moveit_tutorials/doc/realtime_servo/realtime_servo_tutorial.html
```

### 提示:
在使用末端位置控制时请保证机器人在当前姿态下可以进行空间直线运动

在使用过程中请注意速度设置和发送频率以避免造成意外状况