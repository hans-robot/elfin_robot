#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 10 09:21:42 2018

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2018, Han's Robot Co., Ltd.
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
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState

class ElfinModuleCmdPub(object):
    def __init__(self):
        self.action_client=SimpleActionClient('elfin_module_controller/follow_joint_trajectory',
                                              FollowJointTrajectoryAction)
        self.action_goal=FollowJointTrajectoryGoal()
        self.js_sub=rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        self.joint_pos=[]
        
        self.module_joint1_name='elfin_module_joint1'
        self.module_joint2_name='elfin_module_joint2'
        
        self.action_goal.trajectory.joint_names=[self.module_joint1_name, self.module_joint2_name]
        self.action_goal.trajectory.header.stamp.secs=0
        self.action_goal.trajectory.header.stamp.nsecs=0
    
    def cmd_pub(self):
        if len(self.joint_pos)!=2:
            rospy.logwarn("Didn't get joint_states of "+self.module_joint1_name+" and "+self.module_joint2_name)
            return
        
        point_goal=JointTrajectoryPoint()
        point_goal.positions=[self.joint_pos[0]+0.4, self.joint_pos[1]-0.5]
        point_goal.velocities=[0, 0]
        point_goal.accelerations=[0, 0]
        # 注意nsecs为纳秒，而secs为秒
        point_goal.time_from_start=rospy.Time(secs=2, nsecs=0)
        
        self.action_goal.trajectory.points.append(point_goal)
        self.action_client.wait_for_server()
        self.action_client.send_goal(self.action_goal)
        self.action_goal.trajectory.points=[]
    
    def joint_states_cb(self, data):
        if self.module_joint1_name in data.name:
            joint1_index=data.name.index(self.module_joint1_name)
            self.joint_pos.append(data.position[joint1_index])
        
        if self.module_joint2_name in data.name:
            joint2_index=data.name.index(self.module_joint2_name)
            self.joint_pos.append(data.position[joint2_index])
        
        self.js_sub.unregister()
        

if __name__ == "__main__":
    rospy.init_node("elfin_module_cmd_pub", anonymous=True)
    cmd_publisher=ElfinModuleCmdPub()
    rospy.sleep(3)
    cmd_publisher.cmd_pub()
    rospy.spin()

