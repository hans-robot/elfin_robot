/*
Created on Mon Dec 15 10:38:07 2017

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

#ifndef ELFIN_BASIC_API_H
#define ELFIN_BASIC_API_H

#include <elfin_basic_api/elfin_teleop_api.h>
#include <elfin_basic_api/elfin_motion_api.h>
#include <elfin_basic_api/ElfinBasicAPIDynamicReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>

namespace elfin_basic_api {

class ElfinBasicAPI
{
public:
    ElfinBasicAPI(moveit::planning_interface::MoveGroup *group, std::string action_name, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);
    ~ElfinBasicAPI();

    void dynamicReconfigureCallback(ElfinBasicAPIDynamicReconfigureConfig &config, uint32_t level);

    void setVelocityScaling(double data);

    bool setVelocityScaling_cb(elfin_robot_msgs::SetFloat64::Request &req, elfin_robot_msgs::SetFloat64::Response &resp);
    bool updateVelocityScaling_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

private:
    moveit::planning_interface::MoveGroup *group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    ros::NodeHandle root_nh_, local_nh_;

    ElfinTeleopAPI *teleop_api_;
    ElfinMotionAPI *motion_api_;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;
    control_msgs::FollowJointTrajectoryGoal goal_;

    dynamic_reconfigure::Server<ElfinBasicAPIDynamicReconfigureConfig> dynamic_reconfigure_server_;

    double velocity_scaling_;
};

}

#endif
