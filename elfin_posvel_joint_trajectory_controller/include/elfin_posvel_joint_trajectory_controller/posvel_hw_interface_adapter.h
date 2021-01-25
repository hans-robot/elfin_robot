/*
Created on Thu Oct 25 10:24 2018

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
*/
// author: Cong Liu

#ifndef ELFIN_POSVEL_HW_INTERFACE_ADAPTER_H
#define ELFIN_POSVEL_HW_INTERFACE_ADAPTER_H

#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <elfin_hardware_interface/posveltrq_command_interface.h>

template <class State>
class HardwareInterfaceAdapter<elfin_hardware_interface::PosVelTrqJointInterface, State>
{
public:
  HardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<elfin_hardware_interface::PosVelTrqJointHandle>& joint_handles, ros::NodeHandle& /*controller_nh*/)
  {
    joint_handles_ptr_ = &joint_handles;

    return true;
  }

  void starting(const ros::Time& /*time*/) {}
  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         desired_state,
                     const State&         /*state_error*/)
  {
    const unsigned int n_joints = joint_handles_ptr_->size();
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      (*joint_handles_ptr_)[i].setCommand(desired_state.position[i], desired_state.velocity[i], 0);
    }
  }

private:
  std::vector<elfin_hardware_interface::PosVelTrqJointHandle>* joint_handles_ptr_;
};


#endif
