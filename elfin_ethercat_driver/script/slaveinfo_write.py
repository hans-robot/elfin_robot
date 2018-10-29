#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  3 15:59:21 2018

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

import commands
import rospy

class SlaveInfoWrite(object):
    def __init__(self):
        self.param_ns='slaveinfo_write/'
        self.esi_has_path, self.esi_path=commands.getstatusoutput('rospack find elfin_ethercat_driver')
        if self.esi_has_path!=0:
            print self.esi_path
            print 'please try: roscd elfin_ethercat_driver'
            return
                
        if rospy.has_param(self.param_ns+'eth_name'):
            self.eth_name=rospy.get_param(self.param_ns+'eth_name')
        else:
            rospy.logerr('please set '+self.param_ns+'eth_name parameter first')
            return
        
        if rospy.has_param(self.param_ns+'slave_no'):
            self.slave_no=rospy.get_param(self.param_ns+'slave_no')
        else:
            rospy.logerr('please set '+self.param_ns+'slave_no parameter first')
            return
        
        if rospy.has_param(self.param_ns+'esi_file'):
            self.esi_file=rospy.get_param(self.param_ns+'esi_file')
        else:
            rospy.logerr('please set '+self.param_ns+'esi_file parameter first')
            return
        
        if 'backup' in self.esi_file:
            rospy.logerr("esi file's name should not contain 'backup', please rename it")
            return
        
        self.esi_write()
        
    def esi_write(self):
        for i in xrange(len(self.slave_no)):
            backup_name=self.esi_path+'/script/slave'+str(self.slave_no[i])+'_backup.esi'
            write_file_name=self.esi_path+'/script/'+self.esi_file
            print commands.getoutput('eepromtool '+self.eth_name+' '+str(self.slave_no[i])+' -r '+backup_name)
            print commands.getoutput('eepromtool '+self.eth_name+' '+str(self.slave_no[i])+' -w '+write_file_name)

if __name__ == '__main__':
    rospy.init_node('slaveinfo_write')
    SlaveInfoWrite()
    
    
    

