#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import time
from geometry_msgs.msg import TwistStamped,PoseStamped
from control_msgs.msg import JointJog
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, SwitchControllerResponse 


class  test():
    def __init__(self):

        rospy.init_node('servo_test')
        self.elfin_servoPose = rospy.Publisher("/servo_server/delta_twist_cmds", TwistStamped, queue_size=10)
        self.elfin_servoJoint = rospy.Publisher("/servo_server/delta_joint_cmds", JointJog, queue_size=10)
        self.servo_Pose = TwistStamped()
        self.servo_Joint = JointJog()
        self.rate = rospy.Rate(10)
            
    def servo_movePose(self):
        self.servo_Pose.twist.linear.x = 0.1
        self.servo_Pose.twist.linear.y = 0.1
        self.servo_Pose.twist.linear.z = 0.1
        self.servo_Pose.twist.angular.x = 0.0
        self.servo_Pose.twist.angular.y = 0.0
        self.servo_Pose.twist.angular.z = 0.0
        while not rospy.is_shutdown():
            self.servo_Pose.header.stamp = rospy.get_rostime()
            self.elfin_servoPose.publish(self.servo_Pose)
            self.rate.sleep()


    def servo_moveJoint(self):
        
        self.servo_Joint.joint_names = ['elfin_joint1','elfin_joint2','elfin_joint3','elfin_joint4','elfin_joint5','elfin_joint6']
        self.servo_Joint.displacements = [0,0.0,0.0,0.0,0.0,0.0]
        self.servo_Joint.velocities = [0.5,0.0,0.0,0.0,0.0,0.0]

        while not rospy.is_shutdown():
            self.servo_Joint.header.stamp = rospy.get_rostime()
            self.elfin_servoJoint.publish(self.servo_Joint)
            self.rate.sleep()


if __name__=='__main__':  
    test = test()
    test.servo_movePose()
    rospy.spin()
