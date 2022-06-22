#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 27 15:02:10 2017

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2017, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 
"""

# author: Cong Liu
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

class CmdPub(object):
    def __init__(self):
        self.joints_pub=rospy.Publisher('elfin_basic_api/joint_goal', JointState, queue_size=1)
        self.cart_pub=rospy.Publisher('elfin_basic_api/cart_goal', PoseStamped, queue_size=1)
        self.cart_path_pub=rospy.Publisher('elfin_basic_api/cart_path_goal', PoseArray, queue_size=1)
    def function_pub_joints(self):
        js=JointState()
        js.name=['elfin_joint1', 'elfin_joint2', 'elfin_joint3',
                 'elfin_joint4', 'elfin_joint5', 'elfin_joint6']
        js.position=[0.4, 0.4, 0.4, 0.4, 0.4, 0.4]
        js.header.stamp=rospy.get_rostime()
        self.joints_pub.publish(js)

    def function_pub_cart_elfin3(self):
        ps=PoseStamped()
        ps.header.stamp=rospy.get_rostime()
        ps.header.frame_id='elfin_base_link'
        ps.pose.position.x=0.161
        ps.pose.position.y=0.061
        ps.pose.position.z=0.719
        ps.pose.orientation.x=0
        ps.pose.orientation.y=0
        ps.pose.orientation.z=0
        ps.pose.orientation.w=1
        self.cart_pub.publish(ps)
    
    # You'd better run this function when the robot has finished 
    # the motion in function_pub_cart_elfin3()
    def function_pub_cart_path_elfin3(self):
        pa=PoseArray()
        pa.header.stamp=rospy.get_rostime()
        pa.header.frame_id='elfin_base_link'
        
        ps=Pose()
        ps.position.x=0.201
        ps.position.y=0.141
        ps.position.z=0.719
        ps.orientation.x=0
        ps.orientation.y=0
        ps.orientation.z=0
        ps.orientation.w=1
        
        ps1=Pose()
        ps1.position.x=0.241
        ps1.position.y=0.221
        ps1.position.z=0.719
        ps1.orientation.x=0
        ps1.orientation.y=0
        ps1.orientation.z=0
        ps1.orientation.w=1
        
        pa.poses.append(ps)
        pa.poses.append(ps1)
        self.cart_path_pub.publish(pa)
    
    def function_pub_cart_elfin5(self):
        ps=PoseStamped()
        ps.header.stamp=rospy.get_rostime()
        ps.header.frame_id='elfin_base_link'
        ps.pose.position.x=0.258
        ps.pose.position.y=-0.031
        ps.pose.position.z=0.927
        ps.pose.orientation.x=0
        ps.pose.orientation.y=0
        ps.pose.orientation.z=0
        ps.pose.orientation.w=1
        self.cart_pub.publish(ps)

    # You'd better run this function when the robot has finished 
    # the motion in function_pub_cart_elfin5()
    def function_pub_cart_path_elfin5(self):
        pa=PoseArray()
        pa.header.stamp=rospy.get_rostime()
        pa.header.frame_id='elfin_base_link'
        
        ps=Pose()
        ps.position.x=0.308
        ps.position.y=-0.131
        ps.position.z=0.927
        ps.orientation.x=0
        ps.orientation.y=0
        ps.orientation.z=0
        ps.orientation.w=1
        
        ps1=Pose()
        ps1.position.x=0.358
        ps1.position.y=-0.231
        ps1.position.z=0.927
        ps1.orientation.x=0
        ps1.orientation.y=0
        ps1.orientation.z=0
        ps1.orientation.w=1
        
        pa.poses.append(ps)
        pa.poses.append(ps1)
        self.cart_path_pub.publish(pa)
        
    def function_pub_cart_elfin10(self):
        ps=PoseStamped()
        ps.header.stamp=rospy.get_rostime()
        ps.header.frame_id='elfin_base_link'
        ps.pose.position.x=0.204
        ps.pose.position.y=0.005
        ps.pose.position.z=1.143
        ps.pose.orientation.x=0
        ps.pose.orientation.y=0
        ps.pose.orientation.z=0
        ps.pose.orientation.w=1
        self.cart_pub.publish(ps)
        
    # You'd better run this function when the robot has finished 
    # the motion in function_pub_cart_elfin10()
    def function_pub_cart_path_elfin10(self):
        pa=PoseArray()
        pa.header.stamp=rospy.get_rostime()
        pa.header.frame_id='elfin_base_link'
        
        ps=Pose()
        ps.position.x=0.264
        ps.position.y=0.125
        ps.position.z=1.143
        ps.orientation.x=0
        ps.orientation.y=0
        ps.orientation.z=0
        ps.orientation.w=1
        
        ps1=Pose()
        ps1.position.x=0.324
        ps1.position.y=0.245
        ps1.position.z=1.143
        ps1.orientation.x=0
        ps1.orientation.y=0
        ps1.orientation.z=0
        ps1.orientation.w=1
        
        pa.poses.append(ps)
        pa.poses.append(ps1)
        self.cart_path_pub.publish(pa)

if __name__=='__main__':
    rospy.init_node('cmd_pub', anonymous=True)
    cp=CmdPub()
    rospy.sleep(1)
    cp.function_pub_joints()
#    cp.function_pub_cart_elfin3()
#    cp.function_pub_cart_path_elfin3()
#    cp.function_pub_cart_elfin5()
#    cp.function_pub_cart_path_elfin5()
#    cp.function_pub_cart_elfin10()
#    cp.function_pub_cart_path_elfin10()

    rospy.spin()

