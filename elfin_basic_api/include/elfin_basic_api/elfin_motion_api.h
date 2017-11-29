/*
Created on Mon Nov 27 14:24:30 2017

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

#ifndef ELFIN_MOTION_API_H
#define ELFIN_MOTION_API_H

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace elfin_basic_api {

class ElfinMotionAPI
{
public:
    ElfinMotionAPI(moveit::planning_interface::MoveGroup *group, std::string action_name);
    void jointGoalCB(const sensor_msgs::JointStateConstPtr &msg);
    void cartGoalCB(const geometry_msgs::PoseStampedConstPtr &msg);
    bool getRefLink_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    bool getEndLink_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

private:
    moveit::planning_interface::MoveGroup *group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    ros::NodeHandle root_nh_, motion_nh_;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;
    control_msgs::FollowJointTrajectoryGoal goal_;

    ros::Subscriber joint_goal_sub_;
    ros::Subscriber cart_goal_sub_;

    std::string motion_link_;

    ros::ServiceServer get_reference_link_server_;
    ros::ServiceServer get_end_link_server_;

};

}

#endif
