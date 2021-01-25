/*
Created on Thurs Nov 16 09:36:10 2017

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

#include "elfin_basic_api/elfin_basic_api_node.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"elfin_basic_api", ros::init_options::AnonymousName);

    ros::CallbackQueue move_group_cb_queue;

    std::string move_group_name="elfin_arm";
    std::string move_group_desc="robot_description";
    ros::NodeHandle move_group_nh;
    move_group_nh.setCallbackQueue(&move_group_cb_queue);

    moveit::planning_interface::MoveGroupInterface::Options move_group_options(move_group_name, move_group_desc, move_group_nh);

    moveit::planning_interface::MoveGroupInterface move_group(move_group_options);

    ros::AsyncSpinner move_group_spinner(1, &move_group_cb_queue);
    move_group_spinner.start();

    move_group.startStateMonitor();

    move_group.getCurrentJointValues();

    ros::AsyncSpinner common_spinner(1);
    common_spinner.start();

    //boost::shared_ptr<tf::Transformer> tf_ptr(new tf::Transformer());

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startStateMonitor();
    planning_scene_monitor->startWorldGeometryMonitor();

    elfin_basic_api::ElfinBasicAPI basic_api(&move_group, "elfin_arm_controller/follow_joint_trajectory", planning_scene_monitor);

    ros::waitForShutdown();

    return 0;
}

