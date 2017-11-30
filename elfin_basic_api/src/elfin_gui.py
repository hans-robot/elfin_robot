#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 28 12:18:05 2017

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

from __future__ import division
import rospy
import math
import tf
import moveit_commander
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from elfin_robot_msgs.srv import SetInt16, SetInt16Request
import wx
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import threading

class MyFrame(wx.Frame):  
  
    def __init__(self,parent,id):  
        the_size=(700, 470)
        wx.Frame.__init__(self,parent,id,'Elfin Control Panel',pos=(250,100),size=the_size) 
        self.SetMinSize(the_size)
        self.SetMaxSize(the_size)
        self.panel=wx.Panel(self)  
        
        self.listener = tf.TransformListener()
        
        self.robot=moveit_commander.RobotCommander()
        self.scene=moveit_commander.PlanningSceneInterface()
        self.group=moveit_commander.MoveGroupCommander('elfin_arm')

        self.controller_ns='elfin_arm_controller/'
        self.elfin_driver_ns='elfin_ros_control/elfin/'
        
        self.elfin_basic_api_ns='elfin_basic_api/'
        
        self.joint_names=rospy.get_param(self.controller_ns+'joints', [])
                
        self.js_display=[0]*6 # joint_states
        self.jm_button=[0]*6 # joints_minus
        self.jp_button=[0]*6 # joints_plus
        self.js_label=[0]*6 # joint_states
                      
        self.ps_display=[0]*6 # pcs_states
        self.pm_button=[0]*6 # pcs_minus
        self.pp_button=[0]*6 # pcs_plus
        self.ps_label=[0]*6 # pcs_states
                      
        self.display_init()
        self.key=[]
                                
        btn_height=330
                
        self.power_on_btn=wx.Button(self.panel, label='Servo On', name='Servo On',
                                    pos=(20, btn_height), size=(90,40))
        
        self.power_off_btn=wx.Button(self.panel, label='Servo Off', name='Servo Off',
                                    pos=(130, btn_height), size=(90,40))
        
        self.reset_btn=wx.Button(self.panel, label='Clear Fault', 
                                pos=(240, btn_height), size=(100,40))
        
        self.home_btn=wx.Button(self.panel, label='Home', name='home_btn',
                                pos=(360, btn_height), size=(90,40))
        
        self.stop_btn=wx.Button(self.panel, label='Stop', name='Stop',
                                pos=(470, btn_height), size=(90,40))
        
        self.servo_state_label=wx.StaticText(self.panel, label='Servo state:',
                                              pos=(590, btn_height-10))
        self.servo_state_show=wx.TextCtrl(self.panel, style=(wx.TE_CENTER |wx.TE_READONLY),
                                    value='', pos=(600, btn_height+10))
        self.servo_state=bool()
        
        self.fault_state_label=wx.StaticText(self.panel, label='Fault state:',
                                              pos=(590, btn_height+60))
        self.fault_state_show=wx.TextCtrl(self.panel, style=(wx.TE_CENTER |wx.TE_READONLY),
                                    value='', pos=(600, btn_height+80))
        self.fault_state=bool()
        
        self.reply_show_label=wx.StaticText(self.panel, label='Result:',
                                           pos=(20, btn_height+60))
        self.reply_show=wx.TextCtrl(self.panel, style=(wx.TE_CENTER |wx.TE_READONLY),
                                    value='', size=(550, 30), pos=(20, btn_height+80))
        
        self.dlg=wx.Dialog(self.panel, title='messag', size=(200, 50))
        self.dlg.Bind(wx.EVT_CLOSE, self.closewindow)
        self.dlg_panel=wx.Panel(self.dlg)
        self.dlg_label=wx.StaticText(self.dlg_panel, label='hello', pos=(20,20))
                        
        self.call_teleop_joint=rospy.ServiceProxy(self.elfin_basic_api_ns+'joint_teleop', 
                                                  SetInt16)
        self.call_teleop_joint_req=SetInt16Request()
        
        self.call_teleop_cart=rospy.ServiceProxy(self.elfin_basic_api_ns+'cart_teleop', 
                                                 SetInt16)
        self.call_teleop_cart_req=SetInt16Request()
        
        self.call_teleop_stop=rospy.ServiceProxy(self.elfin_basic_api_ns+'stop_teleop', 
                                                 SetBool)
        self.call_teleop_stop_req=SetBoolRequest()
        
        self.call_stop=rospy.ServiceProxy(self.elfin_basic_api_ns+'stop_teleop', 
                                          SetBool)
        self.call_stop_req=SetBoolRequest()
        self.call_stop_req.data=True
        self.stop_btn.Bind(wx.EVT_BUTTON, 
                           lambda evt, cl=self.call_stop,
                           rq=self.call_stop_req :
                           self.call_set_bool_common(evt, cl, rq))
            
        self.call_reset=rospy.ServiceProxy(self.elfin_driver_ns+'clear_fault', SetBool)
        self.call_reset_req=SetBoolRequest()
        self.call_reset_req.data=True
        self.reset_btn.Bind(wx.EVT_BUTTON, 
                           lambda evt, cl=self.call_reset,
                           rq=self.call_reset_req :
                           self.call_set_bool_common(evt, cl, rq))
                
        self.call_power_on=rospy.ServiceProxy(self.elfin_driver_ns+'enable_robot', SetBool)
        self.call_power_on_req=SetBoolRequest()
        self.call_power_on_req.data=True
        self.power_on_btn.Bind(wx.EVT_BUTTON, 
                               lambda evt, cl=self.call_power_on,
                               rq=self.call_power_on_req :
                               self.call_set_bool_common(evt, cl, rq))
        
        self.call_power_off=rospy.ServiceProxy(self.elfin_driver_ns+'disable_robot', SetBool)
        self.call_power_off_req=SetBoolRequest()
        self.call_power_off_req.data=True
        self.power_off_btn.Bind(wx.EVT_BUTTON, 
                               lambda evt, cl=self.call_power_off,
                               rq=self.call_power_off_req :
                               self.call_set_bool_common(evt, cl, rq))
                
        self.call_move_homing=rospy.ServiceProxy(self.elfin_basic_api_ns+'home_teleop', 
                                                 SetBool)
        self.call_move_homing_req=SetBoolRequest()
        self.call_move_homing_req.data=True
        self.home_btn.Bind(wx.EVT_LEFT_DOWN, 
                           lambda evt, cl=self.call_move_homing,
                           rq=self.call_move_homing_req :
                           self.call_set_bool_common(evt, cl, rq))
        self.home_btn.Bind(wx.EVT_LEFT_UP,
                           lambda evt, mark=100:
                           self.release_button(evt, mark) )
            
        self.action_client=SimpleActionClient(self.controller_ns+'follow_joint_trajectory',
                                              FollowJointTrajectoryAction)
        self.action_goal=FollowJointTrajectoryGoal()
        self.action_goal.trajectory.joint_names=self.joint_names
        
    def display_init(self):
        js_pos=[250, 20]
        dis_h=50
        for i in xrange(len(self.js_display)):
            self.js_display[i]=wx.TextCtrl(self.panel, 
                                           style=(wx.TE_CENTER |wx.TE_READONLY),
                                           value='', 
                                           pos=(js_pos[0],
                                                js_pos[1]+(5-i)*dis_h))
            self.js_label[i]=wx.StaticText(self.panel, 
                                           label='J'+str(i+1)+'/deg:',
                                           pos=(js_pos[0]-60,
                                                js_pos[1]+5+(5-i)*dis_h))
            self.jm_button[i]=wx.Button(self.panel,
                                        label='J'+str(i+1)+' -', 
                                        pos=(js_pos[0]-150,
                                             js_pos[1]-5+(5-i)*dis_h),
                                        size=(70,40))
                                        
            self.jm_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=-1*(i+1) : self.teleop_joints(evt, mark) )
            self.jm_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=-1*(i+1) : self.release_button(evt, mark) )
            
            self.jp_button[i]=wx.Button(self.panel,
                                        label='J'+str(i+1)+' +', 
                                        pos=(js_pos[0]-230,
                                             js_pos[1]-5+(5-i)*dis_h),
                                        size=(70,40))
                                        
            self.jp_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=i+1 : self.teleop_joints(evt, mark) )
            self.jp_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=i+1 : self.release_button(evt, mark) )
            
        ps_pos=[600, 20]
        pcs_btn_label=['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        pcs_label=['X', 'Y', 'Z', 'R', 'P', 'Y']
        unit_label=['/mm:', '/mm:', '/mm:', '/deg:', '/deg:', '/deg:']
        for i in xrange(len(self.ps_display)):
            self.ps_display[i]=wx.TextCtrl(self.panel, 
                                           style=(wx.TE_CENTER |wx.TE_READONLY),
                                           value='', 
                                           pos=(ps_pos[0],
                                                ps_pos[1]+(5-i)*dis_h))
            self.ps_label[i]=wx.StaticText(self.panel, 
                                           label=pcs_label[i]+unit_label[i],
                                           pos=(ps_pos[0]-60,
                                                ps_pos[1]+5+(5-i)*dis_h))
            self.pm_button[i]=wx.Button(self.panel,
                                        label=pcs_btn_label[i]+' -', 
                                        pos=(ps_pos[0]-150,
                                             ps_pos[1]-5+(5-i)*dis_h),
                                        size=(70,40))
                                        
            self.pm_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=-1*(i+1) : self.teleop_pcs(evt, mark) )
            self.pm_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=-1*(i+1) : self.release_button(evt, mark) )
            
            self.pp_button[i]=wx.Button(self.panel,
                                        label=pcs_btn_label[i]+' +', 
                                        pos=(ps_pos[0]-230,
                                             ps_pos[1]-5+(5-i)*dis_h),
                                        size=(70,40))
                                        
            self.pp_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=i+1 : self.teleop_pcs(evt, mark) )
            self.pp_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=i+1 : self.release_button(evt, mark) )
        
    def action_stop(self):
        self.action_client.wait_for_server()
        self.action_goal.trajectory.header.stamp.secs=0
        self.action_goal.trajectory.header.stamp.nsecs=0
        self.action_goal.trajectory.points=[]
        self.action_client.send_goal(self.action_goal)
    
    def teleop_joints(self,event,mark):       
        self.call_teleop_joint_req.data=mark
        resp=self.call_teleop_joint.call(self.call_teleop_joint_req)
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
        
    def teleop_pcs(self,event,mark): 
        self.call_teleop_cart_req.data=mark            
        resp=self.call_teleop_cart.call(self.call_teleop_cart_req)
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()    
    
    def release_button(self, event, mark):
        self.call_teleop_stop_req.data=True
        resp=self.call_teleop_stop.call(self.call_teleop_stop_req)
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
    
    def call_set_bool_common(self, event, client, request):
        btn=event.GetEventObject()
        check_list=['Servo On', 'Servo Off']
        if btn.GetName() in check_list:
            self.show_message_dialog(btn.GetName(), client, request)
        else:
            try:
                resp=client.call(request)
                wx.CallAfter(self.update_reply_show, resp)
            except rospy.ServiceException, e:
                resp=SetBoolResponse()
                resp.success=False
                resp.message='no such service in simulation'
                wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
    
    def thread_bg(self, client, request):
        wx.CallAfter(self.show_dialog)
        self.action_stop()
        rospy.sleep(1)
        try:
            resp=client.call(request)
            wx.CallAfter(self.update_reply_show, resp)
        except rospy.ServiceException, e:
            resp=SetBoolResponse()
            resp.success=False
            resp.message='no such service in simulation'
            wx.CallAfter(self.update_reply_show, resp)
        wx.CallAfter(self.destroy_dialog)
    
    def show_message_dialog(self, message, cl, rq):
        msg='executing ['+message+']'
        self.dlg_label.SetLabel(msg)
        t=threading.Thread(target=self.thread_bg, args=(cl, rq,))
        t.start()
        
    def show_dialog(self):
        self.dlg.ShowModal()
        
    def destroy_dialog(self):
        self.dlg.EndModal(0)
        
    def closewindow(self,event):  
#        self.Destroy()  
        pass
    
    def updateDisplay(self, msg):      
        for i in xrange(len(self.js_display)):
            self.js_display[i].SetValue(msg[i])
        
        for i in xrange(len(self.ps_display)):
            self.ps_display[i].SetValue(msg[i+6])
    
    def update_reply_show(self,msg):
        if msg.success:
            self.reply_show.SetBackgroundColour(wx.Colour(200, 225, 200))
        else:
            self.reply_show.SetBackgroundColour(wx.Colour(225, 200, 200))
        self.reply_show.SetValue(msg.message)
            
    def update_servo_state(self, msg):
        if msg.data:
            self.servo_state_show.SetBackgroundColour(wx.Colour(200, 225, 200))
            self.servo_state_show.SetValue('Enabled')
        else:
            self.servo_state_show.SetBackgroundColour(wx.Colour(225, 200, 200))
            self.servo_state_show.SetValue('Disabled')
    
    def update_fault_state(self, msg):
        if msg.data:
            self.fault_state_show.SetBackgroundColour(wx.Colour(225, 200, 200))
            self.fault_state_show.SetValue('Warning')
        else:
            self.fault_state_show.SetBackgroundColour(wx.Colour(200, 225, 200))
            self.fault_state_show.SetValue('No Fault')
    
    
    def js_call_back(self, data):
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform(self.group.get_planning_frame(),
                                               self.group.get_end_effector_link(),
                                               rospy.Time(0), rospy.Duration(100))
                (xyz,qua) = self.listener.lookupTransform(self.group.get_planning_frame(), 
                                                        self.group.get_end_effector_link(), 
                                                        rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        rpy=tf.transformations.euler_from_quaternion(qua)
        
        for i in xrange(len(data.position)):
            self.key.append(str(round(data.position[i]*180/math.pi, 2)))
            
        self.key.append(str(round(xyz[0]*1000, 2)))
        self.key.append(str(round(xyz[1]*1000, 2)))
        self.key.append(str(round(xyz[2]*1000, 2)))
        
        self.key.append(str(round(rpy[0]*180/math.pi, 2)))
        self.key.append(str(round(rpy[1]*180/math.pi, 2)))
        self.key.append(str(round(rpy[2]*180/math.pi, 2)))
        
        wx.CallAfter(self.updateDisplay, self.key)
        self.key=[]
    
    def servo_state_cb(self, data):
        self.servo_state=data.data
        wx.CallAfter(self.update_servo_state, data)
    
    def fault_state_cb(self, data):
        self.fault_state=data.data
        wx.CallAfter(self.update_fault_state, data)
    
    def listen(self):
        rospy.Subscriber('joint_states', JointState, self.js_call_back)
        rospy.Subscriber(self.elfin_driver_ns+'enable_state', Bool, self.servo_state_cb)
        rospy.Subscriber(self.elfin_driver_ns+'fault_state', Bool, self.fault_state_cb)
  
if __name__=='__main__':  
    rospy.init_node('elfin_gui')
    app=wx.PySimpleApp()  
    myframe=MyFrame(parent=None,id=-1)  
    myframe.Show(True)

    myframe.listen()

    app.MainLoop()
