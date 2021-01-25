/*
Created on Mon Nov 13 15:20:10 2017

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

#ifndef ELFIN_TELEOP_API_H
#define ELFIN_TELEOP_API_H

#include <ros/ros.h>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <elfin_robot_msgs/SetInt16.h>
#include <elfin_robot_msgs/SetFloat64.h>
#include <std_msgs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Empty.h>
#include <elfin_basic_api/elfin_basic_api_const.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace elfin_basic_api {

class ElfinTeleopAPI
{
public:
    ElfinTeleopAPI(moveit::planning_interface::MoveGroupInterface *group, std::string action_name, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);
    void teleopJointCmdNoLimitCB(const std_msgs::Int64ConstPtr &msg);

    void teleopJointCmdCB(const std_msgs::Int64ConstPtr &msg);
    void teleopCartCmdCB(const std_msgs::Int64ConstPtr &msg);
    void teleopStopCB(const std_msgs::EmptyConstPtr &msg);

    void setVelocityScaling(double data);
    void setRefFrames(std::string ref_link);
    void setEndFrames(std::string end_link);

    bool jointTeleop_cb(elfin_robot_msgs::SetInt16::Request &req, elfin_robot_msgs::SetInt16::Response &resp);
    bool cartTeleop_cb(elfin_robot_msgs::SetInt16::Request &req, elfin_robot_msgs::SetInt16::Response &resp);
    bool homeTeleop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    bool teleopStop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

    void PoseStampedRotation(geometry_msgs::PoseStamped &pose_stamped, const tf::Vector3 &axis, double angle);

private:
    moveit::planning_interface::MoveGroupInterface *group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    ros::NodeHandle root_nh_, teleop_nh_;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;
    control_msgs::FollowJointTrajectoryGoal goal_;
    ros::Subscriber sub_teleop_joint_command_no_limit_;

    ros::ServiceServer joint_teleop_server_;
    ros::ServiceServer cart_teleop_server_;
    ros::ServiceServer home_teleop_server_;
    ros::ServiceServer teleop_stop_server_;

    double joint_step_;
    double joint_duration_ns_;

    double joint_speed_limit_;
    double velocity_scaling_;
    double joint_speed_default_;
    double cart_duration_default_;
    double resolution_angle_;
    double resolution_linear_;

    double joint_speed_;
    double cart_duration_; // in second

    std::string end_link_;
    std::string reference_link_;
    std::string default_tip_link_;
    std::string root_link_;

    tf::TransformListener tf_listener_;

    tf::StampedTransform transform_rootToRef_;
    tf::StampedTransform transform_tipToEnd_;
};

}

#endif
