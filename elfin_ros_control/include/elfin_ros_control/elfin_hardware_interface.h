/*
Created on Wed Oct 25 11:36:26 2017

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
*/
// author: Cong Liu

#ifndef ELFIN_ROS_CONTROL_HW_INTERFACE
#define ELFIN_ROS_CONTROL_HW_INTERFACE

#include <ros/ros.h>
#include <urdf/model.h>
#include <pthread.h>
#include <time.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllers.h>

#include "elfin_ethercat_driver/elfin_ethercat_driver.h"

namespace elfin_ros_control {

typedef struct{
    std::string name;
    double reduction_ratio;
    double count_rad_factor;
    int32_t count_zero;

    double position;
    double velocity;
    double effort;

    double position_cmd;
    double velocity_cmd;
    double effort_cmd;
}AxisInfo;

typedef struct{
    elfin_ethercat_driver::ElfinEtherCATClient* client_ptr;
    AxisInfo axis1;
    AxisInfo axis2;
}ModuleInfo;

class ElfinHWInterface : public hardware_interface::RobotHW
{
public:
    ElfinHWInterface(elfin_ethercat_driver::EtherCatManager *manager, const ros::NodeHandle &nh=ros::NodeHandle("~"));
    ~ElfinHWInterface();
    void read_init();
    void read_update(const ros::Time &time_now);
    void write_update();

private:
    std::vector<std::string> elfin_driver_names;
    std::vector<elfin_ethercat_driver::ElfinEtherCATDriver*> ethercat_drivers_;
    std::vector<ModuleInfo> module_infos_;
    double C2PI_F; // count to 2 pi factor

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_cmd_interface;

    ros::NodeHandle n_;

    ros::Time read_update_time;
    ros::Duration read_update_dur;
};

}

#endif
