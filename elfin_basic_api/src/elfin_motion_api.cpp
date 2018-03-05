/*
Created on Mon Nov 27 14:22:43 2017

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

#include "elfin_basic_api/elfin_motion_api.h"

namespace elfin_basic_api {

ElfinMotionAPI::ElfinMotionAPI(moveit::planning_interface::MoveGroup *group, std::string action_name):
    group_(group), action_client_(action_name, true), motion_nh_("~")
{
    goal_.trajectory.joint_names=group_->getJointNames();
    goal_.trajectory.header.stamp.sec=0;
    goal_.trajectory.header.stamp.nsec=0;

    joint_goal_sub_=motion_nh_.subscribe("joint_goal", 1, &ElfinMotionAPI::jointGoalCB, this);
    cart_goal_sub_=motion_nh_.subscribe("cart_goal", 1, &ElfinMotionAPI::cartGoalCB, this);
    cart_path_goal_sub_=motion_nh_.subscribe("cart_path_goal", 1, &ElfinMotionAPI::cartPathGoalCB, this);

    get_reference_link_server_=motion_nh_.advertiseService("get_reference_link", &ElfinMotionAPI::getRefLink_cb, this);
    get_end_link_server_=motion_nh_.advertiseService("get_end_link", &ElfinMotionAPI::getEndLink_cb, this);

    motion_link_=group_->getEndEffectorLink();
}

void ElfinMotionAPI::jointGoalCB(const sensor_msgs::JointStateConstPtr &msg)
{
    if(group_->setJointValueTarget(*msg))
    {
        group_->asyncMove();
    }
    else
    {
        ROS_WARN("the robot cannot execute that motion");
    }
}

void ElfinMotionAPI::cartGoalCB(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if(group_->setPoseTarget(*msg, motion_link_))
    {
        group_->asyncMove();
    }
    else
    {
        ROS_WARN("the robot cannot execute that motion");
    }
}

void ElfinMotionAPI::cartPathGoalCB(const geometry_msgs::PoseArrayConstPtr &msg)
{
    moveit_msgs::RobotTrajectory cart_path;
    moveit::planning_interface::MoveGroup::Plan cart_plan;
    group_->setPoseReferenceFrame(msg->header.frame_id);
    double fraction=group_->computeCartesianPath(msg->poses, 0.01, 0.0, cart_path);
    group_->setPoseReferenceFrame(group_->getPlanningFrame());

    if(fraction==-1)
    {
        ROS_WARN("there is an error while computing the cartesian path");
        return;
    }

    if(fraction==1)
    {
        ROS_INFO("the cartesian path can be %.2f%% acheived", fraction * 100.0);
        cart_plan.trajectory_=cart_path;
        group_->asyncExecute(cart_plan);
    }
    else
    {
        ROS_INFO("the cartesian path can only be %.2f%% acheived and it will not be executed", fraction * 100.0);
    }
}

bool ElfinMotionAPI::getRefLink_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    resp.success=true;
    resp.message=group_->getPlanningFrame();
    return true;
}

bool ElfinMotionAPI::getEndLink_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    resp.success=true;
    resp.message=group_->getEndEffectorLink();
    return true;
}

}

