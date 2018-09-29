/*
Created on Fri Sep 28 15:54:17 2018

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

#ifndef ELFIN_HARDWARE_INTERFACE_POSTRQ_COMMAND_INTERFACE_H
#define ELFIN_HARDWARE_INTERFACE_POSTRQ_COMMAND_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace elfin_hardware_interface
{

class PosTrqJointHandle : public hardware_interface::JointStateHandle
{
public:
  PosTrqJointHandle() : hardware_interface::JointStateHandle(), cmd_pos_(0), cmd_trq_(0) {}

  PosTrqJointHandle(const hardware_interface::JointStateHandle& js, double* cmd_pos, double* cmd_trq)
    : hardware_interface::JointStateHandle(js), cmd_pos_(cmd_pos), cmd_trq_(cmd_trq)
  {
    if (!cmd_pos)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command position pointer is null.");
    }
    if (!cmd_trq)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command torque pointer is null.");
    }
  }

  void setCommand(double cmd_pos, double cmd_trq)
  {
    setCommandPosition(cmd_pos);
    setCommandTorque(cmd_trq);
  }

  void setCommandPosition(double cmd_pos)     {assert(cmd_pos_); *cmd_pos_ = cmd_pos;}
  void setCommandTorque(double cmd_trq)     {assert(cmd_trq_); *cmd_trq_ = cmd_trq;}

  double getCommandPosition()     const {assert(cmd_pos_); return *cmd_pos_;}
  double getCommandTorque()     const {assert(cmd_trq_); return *cmd_trq_;}

private:
  double* cmd_pos_;
  double* cmd_trq_;
};


class PosTrqJointInterface : public hardware_interface::HardwareResourceManager<PosTrqJointHandle, hardware_interface::ClaimResources> {};

}

#endif
