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
import os # 20201209: add os path
import tf
import moveit_commander
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from elfin_robot_msgs.srv import SetString, SetStringRequest, SetStringResponse
from elfin_robot_msgs.srv import SetInt16, SetInt16Request
from elfin_robot_msgs.srv import *
import wx
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import threading
import dynamic_reconfigure.client

class MyFrame(wx.Frame):  
  
    def __init__(self,parent,id):  
        the_size=(700, 700) # height from 550 change to 700
        wx.Frame.__init__(self,parent,id,'Elfin Control Panel',pos=(250,100)) 
        self.panel=wx.Panel(self)
        font=self.panel.GetFont()
        font.SetPixelSize((12, 24))
        self.panel.SetFont(font)
        
        self.listener = tf.TransformListener()
        
        self.robot=moveit_commander.RobotCommander()
        self.scene=moveit_commander.PlanningSceneInterface()
        self.group=moveit_commander.MoveGroupCommander('elfin_arm')

        self.controller_ns='elfin_arm_controller/'
        self.elfin_driver_ns='elfin_ros_control/elfin/'
        self.elfin_IO_ns='elfin_ros_control/elfin/io_port1/' # 20201126: add IO ns

        self.call_read_do_req = ElfinIODReadRequest()
        self.call_read_di_req = ElfinIODReadRequest()
        self.call_read_do_req.data = True
        self.call_read_di_req.data = True
        self.call_read_do = rospy.ServiceProxy(self.elfin_IO_ns+'read_do',ElfinIODRead)
        self.call_read_di = rospy.ServiceProxy(self.elfin_IO_ns+'read_di',ElfinIODRead)
        # 20201126: add service for write_do
        self.call_write_DO=rospy.ServiceProxy(self.elfin_IO_ns+'write_do',ElfinIODWrite)
        
        self.elfin_basic_api_ns='elfin_basic_api/'
        
        self.joint_names=rospy.get_param(self.controller_ns+'joints', [])
        
        self.ref_link_name=self.group.get_planning_frame()
        self.end_link_name=self.group.get_end_effector_link()
        
        self.ref_link_lock=threading.Lock()
        self.end_link_lock=threading.Lock()
        self.DO_btn_lock = threading.Lock() # 20201208: add the threading lock
        self.DI_show_lock = threading.Lock()
                
        self.js_display=[0]*6 # joint_states
        self.jm_button=[0]*6 # joints_minus
        self.jp_button=[0]*6 # joints_plus
        self.js_label=[0]*6 # joint_states
                      
        self.ps_display=[0]*6 # pcs_states
        self.pm_button=[0]*6 # pcs_minus
        self.pp_button=[0]*6 # pcs_plus
        self.ps_label=[0]*6 # pcs_states

        # 20201208: add the button array
        self.DO_btn_display=[0]*4 # DO states
        self.DI_display=[0]*4 # DI states
        self.LED_display=[0]*4 # LED states
        self.End_btn_display=[0]*4 # end button states

        self.btn_height=370 # 20201126: from 390 change to 370
        self.btn_path = os.path.dirname(os.path.realpath(__file__)) # 20201209: get the elfin_gui.py path
        btn_lengths=[]
        self.DO_DI_btn_length=[0,92,157,133] # 20201209: the length come from servo on, servo off, home, stop button
        self.btn_interstice=22 # 20201209: come from btn_interstice

        self.display_init()              
        self.key=[]
        self.DO_btn=[0,0,0,0,0,0,0,0] # DO state, first four bits is DO, the other is LED
        self.DI_show=[0,0,0,0,0,0,0,0] # DI state, first four bits is DI, the other is the end button
                
        self.power_on_btn=wx.Button(self.panel, label=' Servo On ', name='Servo On',
                                    pos=(20, self.btn_height))
        btn_lengths.append(self.power_on_btn.GetSize()[0])
        btn_total_length=btn_lengths[0]
        
        self.power_off_btn=wx.Button(self.panel, label=' Servo Off ', name='Servo Off')
        btn_lengths.append(self.power_off_btn.GetSize()[0])
        btn_total_length+=btn_lengths[1]
        
        self.reset_btn=wx.Button(self.panel, label=' Clear Fault ', name='Clear Fault')
        btn_lengths.append(self.reset_btn.GetSize()[0])
        btn_total_length+=btn_lengths[2]

        self.home_btn=wx.Button(self.panel, label='Home', name='home_btn')
        btn_lengths.append(self.home_btn.GetSize()[0])
        btn_total_length+=btn_lengths[3]
        
        self.stop_btn=wx.Button(self.panel, label='Stop', name='Stop')
        btn_lengths.append(self.stop_btn.GetSize()[0])
        btn_total_length+=btn_lengths[4]

        self.btn_interstice=(550-btn_total_length)/4
        btn_pos_tmp=btn_lengths[0]+self.btn_interstice+20 # 20201126: 20:init length + btn0 length + btn_inter:gap
        self.power_off_btn.SetPosition((btn_pos_tmp, self.btn_height))
        
        btn_pos_tmp+=btn_lengths[1]+self.btn_interstice
        self.reset_btn.SetPosition((btn_pos_tmp, self.btn_height))
        
        btn_pos_tmp+=btn_lengths[2]+self.btn_interstice
        self.home_btn.SetPosition((btn_pos_tmp, self.btn_height))
        
        btn_pos_tmp+=btn_lengths[3]+self.btn_interstice
        self.stop_btn.SetPosition((btn_pos_tmp, self.btn_height))
        
        self.servo_state_label=wx.StaticText(self.panel, label='Servo state:',
                                              pos=(590, self.btn_height-10))
        self.servo_state_show=wx.TextCtrl(self.panel, style=(wx.TE_CENTER |wx.TE_READONLY),
                                    value='', pos=(600, self.btn_height+10))
        self.servo_state=bool()
        
        self.servo_state_lock=threading.Lock()
        
        self.fault_state_label=wx.StaticText(self.panel, label='Fault state:',
                                              pos=(590, self.btn_height+60))
        self.fault_state_show=wx.TextCtrl(self.panel, style=(wx.TE_CENTER |wx.TE_READONLY),
                                    value='', pos=(600, self.btn_height+80))
        self.fault_state=bool()
        
        self.fault_state_lock=threading.Lock()

        # 20201209: add the description of end button
        self.end_button_state_label=wx.StaticText(self.panel, label='END Button state',
                                            pos=(555,self.btn_height+172))
        
        self.reply_show_label=wx.StaticText(self.panel, label='Result:',
                                           pos=(20, self.btn_height+260)) # 20201126: btn_height from 120 change to 260.
        self.reply_show=wx.TextCtrl(self.panel, style=(wx.TE_CENTER |wx.TE_READONLY),
                                    value='', size=(670, 30), pos=(20, self.btn_height+280))# 20201126: btn_height from 140 change to 280.
        
        link_textctrl_length=(btn_pos_tmp-40)/2
        
        self.ref_links_show_label=wx.StaticText(self.panel, label='Ref. link:',
                                                    pos=(20, self.btn_height+210)) # 20201126: btn_height from 60 change to 210.
        
        self.ref_link_show=wx.TextCtrl(self.panel, style=(wx.TE_READONLY),
                                           value=self.ref_link_name, size=(link_textctrl_length, 30),
                                           pos=(20, self.btn_height+230)) # 20201126: btn_height from 80 change to 230.
        
        self.end_link_show_label=wx.StaticText(self.panel, label='End link:',
                                               pos=(link_textctrl_length+30, self.btn_height+210))# 20201126: btn_height from 80 change to 200.
        
        self.end_link_show=wx.TextCtrl(self.panel, style=(wx.TE_READONLY),
                                       value=self.end_link_name, size=(link_textctrl_length, 30),
                                       pos=(link_textctrl_length+30, self.btn_height+230))
        
        self.set_links_btn=wx.Button(self.panel, label='Set links', name='Set links')
        self.set_links_btn.SetPosition((btn_pos_tmp, self.btn_height+230)) # 20201126: btn_height from 75 change to 220.
        
        # the variables about velocity scaling
        velocity_scaling_init=rospy.get_param(self.elfin_basic_api_ns+'velocity_scaling',
                                              default=0.4)
        default_velocity_scaling=str(round(velocity_scaling_init, 2))
        self.velocity_setting_label=wx.StaticText(self.panel, label='Velocity Scaling',
                                                  pos=(20, self.btn_height-55)) # 20201126: btn_height from 70 change to 55
        self.velocity_setting=wx.Slider(self.panel, value=int(velocity_scaling_init*100),
                                        minValue=1, maxValue=100,
                                        style = wx.SL_HORIZONTAL,
                                        size=(500, 30),
                                        pos=(45, self.btn_height-35)) # 20201126: btn_height from 70 change to 35
        self.velocity_setting_txt_lower=wx.StaticText(self.panel, label='1%',
                                                    pos=(20, self.btn_height-35)) # 20201126: btn_height from 45 change to 35
        self.velocity_setting_txt_upper=wx.StaticText(self.panel, label='100%',
                                                    pos=(550, self.btn_height-35))# 20201126: btn_height from 45 change to 35
        self.velocity_setting_show=wx.TextCtrl(self.panel, 
                                               style=(wx.TE_CENTER|wx.TE_READONLY), 
                                                value=default_velocity_scaling,
                                                pos=(600, self.btn_height-45))# 20201126: btn_height from 55 change to 45
        self.velocity_setting.Bind(wx.EVT_SLIDER, self.velocity_setting_cb)
        self.teleop_api_dynamic_reconfig_client=dynamic_reconfigure.client.Client(self.elfin_basic_api_ns,
                                                                                  config_callback=self.basic_api_reconfigure_cb)
        
        self.dlg=wx.Dialog(self.panel, title='messag')
        self.dlg.Bind(wx.EVT_CLOSE, self.closewindow)
        self.dlg_panel=wx.Panel(self.dlg)
        self.dlg_label=wx.StaticText(self.dlg_panel, label='hello', pos=(15, 15))
        
        self.set_links_dlg=wx.Dialog(self.panel, title='Set links', size=(400, 100))
        self.set_links_dlg_panel=wx.Panel(self.set_links_dlg)
        
        self.sld_ref_link_show=wx.TextCtrl(self.set_links_dlg_panel, style=wx.TE_PROCESS_ENTER,
                                           value='', pos=(20, 20), size=(link_textctrl_length, 30))
        self.sld_end_link_show=wx.TextCtrl(self.set_links_dlg_panel, style=wx.TE_PROCESS_ENTER,
                                           value='', pos=(20, 70), size=(link_textctrl_length, 30))
        
        self.sld_set_ref_link_btn=wx.Button(self.set_links_dlg_panel, label='Update ref. link',
                                            name='Update ref. link')
        self.sld_set_ref_link_btn.SetPosition((link_textctrl_length+30, 15))
        self.sld_set_end_link_btn=wx.Button(self.set_links_dlg_panel, label='Update end link',
                                            name='Update end link')
        self.sld_set_end_link_btn.SetPosition((link_textctrl_length+30, 65))
        
        self.set_links_dlg.SetSize((link_textctrl_length+self.sld_set_ref_link_btn.GetSize()[0]+50, 120))
        
                        
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
                
        self.call_power_on=rospy.ServiceProxy(self.elfin_basic_api_ns+'enable_robot', SetBool)
        self.call_power_on_req=SetBoolRequest()
        self.call_power_on_req.data=True
        self.power_on_btn.Bind(wx.EVT_BUTTON, 
                               lambda evt, cl=self.call_power_on,
                               rq=self.call_power_on_req :
                               self.call_set_bool_common(evt, cl, rq))
        
        self.call_power_off=rospy.ServiceProxy(self.elfin_basic_api_ns+'disable_robot', SetBool)
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
            
        self.call_set_ref_link=rospy.ServiceProxy(self.elfin_basic_api_ns+'set_reference_link', SetString)
        self.call_set_end_link=rospy.ServiceProxy(self.elfin_basic_api_ns+'set_end_link', SetString)
        self.set_links_btn.Bind(wx.EVT_BUTTON, self.show_set_links_dialog)
        
        self.sld_set_ref_link_btn.Bind(wx.EVT_BUTTON, self.update_ref_link)
        self.sld_set_end_link_btn.Bind(wx.EVT_BUTTON, self.update_end_link)
        
        self.sld_ref_link_show.Bind(wx.EVT_TEXT_ENTER, self.update_ref_link)
        self.sld_end_link_show.Bind(wx.EVT_TEXT_ENTER, self.update_end_link)
            
        self.action_client=SimpleActionClient(self.controller_ns+'follow_joint_trajectory',
                                              FollowJointTrajectoryAction)
        self.action_goal=FollowJointTrajectoryGoal()
        self.action_goal.trajectory.joint_names=self.joint_names
        
        self.SetMinSize(the_size)
        self.SetMaxSize(the_size)
        
    def display_init(self):
        js_pos=[20, 20]
        js_btn_length=[70, 70, 61, 80]
        js_distances=[10, 20, 10, 26]
        dis_h=50
        for i in xrange(len(self.js_display)):
            self.jp_button[i]=wx.Button(self.panel,
                                        label='J'+str(i+1)+' +', 
                                        pos=(js_pos[0],
                                             js_pos[1]+(5-i)*dis_h),
                                        size=(70,40))
            dis_tmp=js_btn_length[0]+js_distances[0]
                                        
            self.jp_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=i+1 : self.teleop_joints(evt, mark) )
            self.jp_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=i+1 : self.release_button(evt, mark) )
            
            self.jm_button[i]=wx.Button(self.panel,
                                        label='J'+str(i+1)+' -', 
                                        pos=(js_pos[0]+dis_tmp,
                                             js_pos[1]+(5-i)*dis_h),
                                        size=(70,40))
            dis_tmp+=js_btn_length[1]+js_distances[1]
                                        
            self.jm_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=-1*(i+1) : self.teleop_joints(evt, mark) )
            self.jm_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=-1*(i+1) : self.release_button(evt, mark) )
            
            pos_js_label=(js_pos[0]+dis_tmp, js_pos[1]+(5-i)*dis_h)
            self.js_label[i]=wx.StaticText(self.panel,
                                           label='J'+str(i+1)+'/deg:',
                                           pos=pos_js_label)
            self.js_label[i].SetPosition((pos_js_label[0], pos_js_label[1]+abs(40-self.js_label[i].GetSize()[1])/2))
            dis_tmp+=js_btn_length[2]+js_distances[2]

            pos_js_display=(js_pos[0]+dis_tmp, js_pos[1]+(5-i)*dis_h)
            self.js_display[i]=wx.TextCtrl(self.panel, 
                                           style=(wx.TE_CENTER |wx.TE_READONLY),
                                           value=' 0000.00 ', 
                                           pos=pos_js_display)
            self.js_display[i].SetPosition((pos_js_display[0], pos_js_display[1]+abs(40-self.js_display[i].GetSize()[1])/2))
            dis_tmp+=js_btn_length[3]+js_distances[3]

        ps_pos=[js_pos[0]+dis_tmp, 20]
        ps_btn_length=[70, 70, 53, 80]
        ps_distances=[10, 20, 10, 20]
        pcs_btn_label=['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        pcs_label=['X', 'Y', 'Z', 'R', 'P', 'Y']
        unit_label=['/mm:', '/mm:', '/mm:', '/deg:', '/deg:', '/deg:']
        for i in xrange(len(self.ps_display)):
            self.pp_button[i]=wx.Button(self.panel,
                                        label=pcs_btn_label[i]+' +', 
                                        pos=(ps_pos[0],
                                             ps_pos[1]+(5-i)*dis_h),
                                        size=(70,40))
            dis_tmp=ps_btn_length[0]+ps_distances[0]
                                        
            self.pp_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=i+1 : self.teleop_pcs(evt, mark) )
            self.pp_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=i+1 : self.release_button(evt, mark) )
            
            self.pm_button[i]=wx.Button(self.panel,
                                        label=pcs_btn_label[i]+' -', 
                                        pos=(ps_pos[0]+dis_tmp,
                                             ps_pos[1]+(5-i)*dis_h),
                                        size=(70,40))
            dis_tmp+=ps_btn_length[1]+ps_distances[1]
                                        
            self.pm_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=-1*(i+1) : self.teleop_pcs(evt, mark) )
            self.pm_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=-1*(i+1) : self.release_button(evt, mark) )
            
            pos_ps_label=(ps_pos[0]+dis_tmp, ps_pos[1]+(5-i)*dis_h)
            self.ps_label[i]=wx.StaticText(self.panel, 
                                           label=pcs_label[i]+unit_label[i],
                                           pos=pos_ps_label)
            self.ps_label[i].SetPosition((pos_ps_label[0], pos_ps_label[1]+abs(40-self.ps_label[i].GetSize()[1])/2))
            dis_tmp+=ps_btn_length[2]+ps_distances[2]
            
            pos_ps_display=(ps_pos[0]+dis_tmp, ps_pos[1]+(5-i)*dis_h)
            self.ps_display[i]=wx.TextCtrl(self.panel, 
                                           style=(wx.TE_CENTER |wx.TE_READONLY),
                                           value='', 
                                           pos=pos_ps_display)
            self.ps_display[i].SetPosition((pos_ps_display[0], pos_ps_display[1]+abs(40-self.ps_display[i].GetSize()[1])/2))
            dis_tmp+=ps_btn_length[3]+ps_distances[3]

        # 20201209: add the DO,LED,DI,end button.
        for i in xrange(len(self.DO_btn_display)):
            self.DO_btn_display[i]=wx.Button(self.panel,label='DO'+str(i),
                                        pos=(20+(self.DO_DI_btn_length[i]+self.btn_interstice)*i,
                                        self.btn_height+40))
            self.DO_btn_display[i].Bind(wx.EVT_BUTTON,
                                    lambda evt,marker=i,cl=self.call_write_DO : 
                                    self.call_write_DO_command(evt,marker,cl))

            self.DI_display[i]=wx.TextCtrl(self.panel, style=(wx.TE_CENTER | wx.TE_READONLY), value='DI'+str(i),
                                size=(self.DO_btn_display[i].GetSize()), 
                                pos=(20+(self.DO_DI_btn_length[i]+self.btn_interstice)*i,self.btn_height+80))

            self.LED_display[i]=wx.Button(self.panel,label='LED'+str(i),
                                        pos=(20+(self.DO_DI_btn_length[i]+self.btn_interstice)*i,self.btn_height+120))
            self.LED_display[i].Bind(wx.EVT_BUTTON,
                                    lambda evt, marker=4+i, cl=self.call_write_DO : 
                                    self.call_write_DO_command(evt, marker,cl))

            png=wx.Image(self.btn_path+'/btn_icon/End_btn'+str(i)+'_low.png',wx.BITMAP_TYPE_PNG).ConvertToBitmap()
            self.End_btn_display[i]=wx.StaticBitmap(self.panel,-1,png,
                                                pos=(40+(self.DO_DI_btn_length[i]+self.btn_interstice)*i,
                                                self.btn_height+160))
    
    def velocity_setting_cb(self, event):
        current_velocity_scaling=self.velocity_setting.GetValue()*0.01
        self.teleop_api_dynamic_reconfig_client.update_configuration({'velocity_scaling': current_velocity_scaling})
        wx.CallAfter(self.update_velocity_scaling_show, current_velocity_scaling)
    
    def basic_api_reconfigure_cb(self, config):
        if self.velocity_setting_show.GetValue()!=config.velocity_scaling:
            self.velocity_setting.SetValue(int(config.velocity_scaling*100))
            wx.CallAfter(self.update_velocity_scaling_show, config.velocity_scaling)        
    
    def action_stop(self):
        self.action_client.wait_for_server(timeout=rospy.Duration(secs=0.5))
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
        check_list=['Servo On', 'Servo Off', 'Clear Fault']
        
        # Check servo state
        if btn.GetName()=='Servo On':
            servo_enabled=bool()
            if self.servo_state_lock.acquire():
                servo_enabled=self.servo_state
                self.servo_state_lock.release()
            if servo_enabled:
                resp=SetBoolResponse()
                resp.success=False
                resp.message='Robot is already enabled'
                wx.CallAfter(self.update_reply_show, resp)
                event.Skip()
                return
        
        # Check fault state
        if btn.GetName()=='Clear Fault':
            fault_flag=bool()
            if self.fault_state_lock.acquire():
                fault_flag=self.fault_state
                self.fault_state_lock.release()
            if not fault_flag:
                resp=SetBoolResponse()
                resp.success=False
                resp.message='There is no fault now'
                wx.CallAfter(self.update_reply_show, resp)
                event.Skip()
                return
        
        # Check if the button is in check list
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
    
    def thread_bg(self, msg, client, request):
        wx.CallAfter(self.show_dialog)
        if msg=='Servo Off':
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

    # 20201201: add function for processing value to DO_btn
    def process_DO_btn(self,value):
        if self.DO_btn_lock.acquire():
            for i in range(0,8):
                tmp = (value >> (12 + i)) & 0x01
                self.DO_btn[i]=tmp
            self.DO_btn_lock.release()

    # 20201201: add function to read DO.
    def call_read_DO_command(self):
        try:
            client = self.call_read_do
            val = client.call(self.call_read_do_req).digital_input
            self.process_DO_btn(val)
        except rospy.ServiceException, e:
            resp=ElfinIODReadResponse()
            resp.digital_input=0x0000

    # 20201201: add function for processing value
    def process_DI_btn(self,value):
        if self.DI_show_lock.acquire():
            if value > 0:
                for i in range(0,8):
                    tmp = (value >> (16 + i)) & 0x01
                    self.DI_show[i]=tmp
            else:
                self.DI_show = [0,0,0,0,0,0,0,0]
        self.DI_show_lock.release()
    
    # 20201201: add function to read DI.
    def call_read_DI_command(self):
        try:
            client = self.call_read_di
            val = client.call(self.call_read_di_req).digital_input
            self.process_DI_btn(val)
        except rospy.ServiceException, e:
            resp=ElfinIODReadResponse()
            resp.digital_input=0x0000

    # 20201202: add function to read DO and DI.
    def monitor_DO_DI(self,evt):
        self.call_read_DI_command()
        self.call_read_DO_command()

    # 20201126: add function to write DO.
    def call_write_DO_command(self, event, marker, client):
        self.justification_DO_btn(marker)
        request = 0
        try:
            self.DO_btn_lock.acquire()
            for i in range(0,8):
                request = request + self.DO_btn[i]*pow(2,i)
            resp=client.call(request << 12)
            self.DO_btn_lock.release()
        except rospy.ServiceException, e:
            self.DO_btn_lock.release()
            resp=ElfinIODWriteResponse()
            resp.success=False
            self.justification_DO_btn(marker)
            rp=SetBoolResponse()
            rp.success=False
            rp.message='no such service for DO control'
            wx.CallAfter(self.update_reply_show, rp)

    # 20201127: add justification to DO_btn
    def justification_DO_btn(self,marker):
        self.DO_btn_lock.acquire()
        if 0 == self.DO_btn[marker]:
            self.DO_btn[marker] = 1
        else:
             self.DO_btn[marker] = 0
        self.DO_btn_lock.release()

    # 20201201: add function to set DO_btn colour
    def set_DO_btn_colour(self):
        self.DO_btn_lock.acquire()
        for i in range(0,4):
            if 0 == self.DO_btn[i]:
                self.DO_btn_display[i].SetBackgroundColour(wx.NullColour)
            else:
                self.DO_btn_display[i].SetBackgroundColour(wx.Colour(200,225,200))
        self.DO_btn_lock.release()

    # 20201201: add function to set DI_show colour
    def set_DI_show_colour(self):
        self.DI_show_lock.acquire()
        for i in range(0,4):
            if 0 == self.DI_show[i]:
                self.DI_display[i].SetBackgroundColour(wx.NullColour)
            else:
                self.DI_display[i].SetBackgroundColour(wx.Colour(200,225,200))
        self.DI_show_lock.release()

    # 20201207: add function to set LED colour
    def set_LED_show_colour(self):
        self.DO_btn_lock.acquire()
        for i in range(4,8):
            if 0 == self.DO_btn[i]:
                self.LED_display[i-4].SetBackgroundColour(wx.NullColour)
            else:
                self.LED_display[i-4].SetBackgroundColour(wx.Colour(200,225,200))
        self.DO_btn_lock.release()

    # 20201207: add function to set End_btn colour
    def set_End_btn_colour(self):
        self.DI_show_lock.acquire()
        for i in range(4,8):
            if 0 == self.DI_show[i]:
                png=wx.Image(self.btn_path+'/btn_icon/End_btn'+str(i-4)+'_low.png',wx.BITMAP_TYPE_PNG)
                self.End_btn_display[i-4].SetBitmap(wx.BitmapFromImage(png))
            else:
                png=wx.Image(self.btn_path+'/btn_icon/End_btn'+str(i-4)+'_high.png',wx.BITMAP_TYPE_PNG)
                self.End_btn_display[i-4].SetBitmap(wx.BitmapFromImage(png))
        self.DI_show_lock.release()

    def set_color(self, evt):
        wx.CallAfter(self.set_DO_btn_colour)
        wx.CallAfter(self.set_DI_show_colour)
        wx.CallAfter(self.set_LED_show_colour)
        wx.CallAfter(self.set_End_btn_colour)
    
    def show_message_dialog(self, message, cl, rq):
        msg='executing ['+message+']'
        self.dlg_label.SetLabel(msg)
        lable_size=[]
        lable_size.append(self.dlg_label.GetSize()[0])
        lable_size.append(self.dlg_label.GetSize()[1])
        self.dlg.SetSize((lable_size[0]+30, lable_size[1]+30))
        t=threading.Thread(target=self.thread_bg, args=(message, cl, rq,))
        t.start()
        
    def show_dialog(self):
        self.dlg.SetPosition((self.GetPosition()[0]+250,
                              self.GetPosition()[1]+250))
        self.dlg.ShowModal()
        
    def destroy_dialog(self):
        self.dlg.EndModal(0)
        
    def closewindow(self,event):
        pass
    
    def show_set_links_dialog(self, evt):
        self.sld_ref_link_show.SetValue(self.ref_link_name)
        self.sld_end_link_show.SetValue(self.end_link_name)
        self.set_links_dlg.SetPosition((self.GetPosition()[0]+150,
                                        self.GetPosition()[1]+250))
        self.set_links_dlg.ShowModal()
    
    def update_ref_link(self, evt):
        request=SetStringRequest()
        request.data=self.sld_ref_link_show.GetValue()
        
        resp=self.call_set_ref_link.call(request)
        wx.CallAfter(self.update_reply_show, resp)
    
    def update_end_link(self, evt):
        request=SetStringRequest()
        request.data=self.sld_end_link_show.GetValue()
        
        resp=self.call_set_end_link.call(request)
        wx.CallAfter(self.update_reply_show, resp)
    
    def updateDisplay(self, msg):      
        for i in xrange(len(self.js_display)):
            self.js_display[i].SetValue(msg[i])
        
        for i in xrange(len(self.ps_display)):
            self.ps_display[i].SetValue(msg[i+6])
            
        if self.ref_link_lock.acquire():
            ref_link=self.ref_link_name
            self.ref_link_lock.release()
        
        if self.end_link_lock.acquire():
            end_link=self.end_link_name
            self.end_link_lock.release()
        
        self.ref_link_show.SetValue(ref_link)
        self.end_link_show.SetValue(end_link)
    
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
        
    def update_velocity_scaling_show(self, msg):
        self.velocity_setting_show.SetValue(str(round(msg, 2)*100)+'%') # 20201127: change the show format
    
    
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
    
    def monitor_status(self, evt):
        self.key=[]
        
        current_joint_values=self.group.get_current_joint_values()
        for i in xrange(len(current_joint_values)):
            self.key.append(str(round(current_joint_values[i]*180/math.pi, 2)))
        
        if self.ref_link_lock.acquire():
            ref_link=self.ref_link_name
            self.ref_link_lock.release()
        
        if self.end_link_lock.acquire():
            end_link=self.end_link_name
            self.end_link_lock.release()
        
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform(ref_link, end_link, rospy.Time(0), rospy.Duration(100))
                (xyz,qua) = self.listener.lookupTransform(ref_link, end_link, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
        rpy=tf.transformations.euler_from_quaternion(qua)
            
        self.key.append(str(round(xyz[0]*1000, 2)))
        self.key.append(str(round(xyz[1]*1000, 2)))
        self.key.append(str(round(xyz[2]*1000, 2)))
        
        self.key.append(str(round(rpy[0]*180/math.pi, 2)))
        self.key.append(str(round(rpy[1]*180/math.pi, 2)))
        self.key.append(str(round(rpy[2]*180/math.pi, 2)))
        
        wx.CallAfter(self.updateDisplay, self.key)
            
    def servo_state_cb(self, data):
        if self.servo_state_lock.acquire():
            self.servo_state=data.data
            self.servo_state_lock.release()
        wx.CallAfter(self.update_servo_state, data)
    
    def fault_state_cb(self, data):
        if self.fault_state_lock.acquire():
            self.fault_state=data.data
            self.fault_state_lock.release()
        wx.CallAfter(self.update_fault_state, data)
    
    def ref_link_name_cb(self, data):
        if self.ref_link_lock.acquire():
            self.ref_link_name=data.data
            self.ref_link_lock.release()
    
    def end_link_name_cb(self, data):
        if self.end_link_lock.acquire():
            self.end_link_name=data.data
            self.end_link_lock.release()
        
    def listen(self):
        rospy.Subscriber(self.elfin_driver_ns+'enable_state', Bool, self.servo_state_cb)
        rospy.Subscriber(self.elfin_driver_ns+'fault_state', Bool, self.fault_state_cb)
        rospy.Subscriber(self.elfin_basic_api_ns+'reference_link_name', String, self.ref_link_name_cb)
        rospy.Subscriber(self.elfin_basic_api_ns+'end_link_name', String, self.end_link_name_cb)

        rospy.Timer(rospy.Duration(nsecs=50000000), self.monitor_DO_DI)
        rospy.Timer(rospy.Duration(nsecs=50000000), self.set_color)
        rospy.Timer(rospy.Duration(nsecs=50000000), self.monitor_status)
  
if __name__=='__main__':  
    rospy.init_node('elfin_gui')
    app=wx.App(False)  
    myframe=MyFrame(parent=None,id=-1)  
    myframe.Show(True)

    myframe.listen()

    app.MainLoop()
