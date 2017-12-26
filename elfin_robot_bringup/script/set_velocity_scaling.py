#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 16 19:59:24 2017

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
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from dynamic_reconfigure.msg import DoubleParameter, Config

class SetVelocityScaling(object):
    def __init__(self):
        self.request=ReconfigureRequest()
        self.velocity_scaling_goal=0.6
        self.elfin_basic_api_ns='elfin_basic_api/'
        self.set_parameters_client=rospy.ServiceProxy(self.elfin_basic_api_ns+'set_parameters',
                                                      Reconfigure)
    
    def set_parameters(self):
        config_empty=Config()
        
        velocity_scaling_param_tmp=DoubleParameter()
        velocity_scaling_param_tmp.name='velocity_scaling'
        velocity_scaling_param_tmp.value=self.velocity_scaling_goal
        self.request.config.doubles.append(velocity_scaling_param_tmp)
        self.set_parameters_client.call(self.request)
        self.request.config=config_empty
    
if __name__ == "__main__":
    rospy.init_node('set_velocity_scaling', anonymous=True)
    svc=SetVelocityScaling()
    svc.set_parameters()
    rospy.spin()

