/*
Created on Mon Dec 15 10:58:42 2017

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

#include <elfin_basic_api/elfin_basic_api.h>

namespace elfin_basic_api {

ElfinBasicAPI::ElfinBasicAPI(moveit::planning_interface::MoveGroupInterface *group, std::string action_name, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor):
    group_(group), action_client_(action_name, true), planning_scene_monitor_(planning_scene_monitor), local_nh_("~")
{
    teleop_api_=new ElfinTeleopAPI(group, action_name, planning_scene_monitor);
    motion_api_=new ElfinMotionAPI(group, action_name, planning_scene_monitor);

    dynamic_reconfigure_server_.setCallback(boost::bind(&ElfinBasicAPI::dynamicReconfigureCallback, this, _1, _2));

    set_ref_link_server_=local_nh_.advertiseService("set_reference_link", &ElfinBasicAPI::setRefLink_cb, this);
    set_end_link_server_=local_nh_.advertiseService("set_end_link", &ElfinBasicAPI::setEndLink_cb, this);
    enable_robot_server_=local_nh_.advertiseService("enable_robot", &ElfinBasicAPI::enableRobot_cb, this);
    disable_robot_server_=local_nh_.advertiseService("disable_robot", &ElfinBasicAPI::disableRobot_cb, this);

    elfin_controller_name_=local_nh_.param<std::string>("controller_name", "elfin_arm_controller");

    switch_controller_client_=root_nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    list_controllers_client_=root_nh_.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
    get_motion_state_client_=root_nh_.serviceClient<std_srvs::SetBool>("/elfin_ros_control/elfin/get_motion_state");
    get_pos_align_state_client_=root_nh_.serviceClient<std_srvs::SetBool>("/elfin_ros_control/elfin/get_pos_align_state");
    raw_enable_robot_client_=root_nh_.serviceClient<std_srvs::SetBool>("/elfin_ros_control/elfin/enable_robot");
    raw_disable_robot_client_=root_nh_.serviceClient<std_srvs::SetBool>("/elfin_ros_control/elfin/disable_robot");

    ref_link_name_publisher_=local_nh_.advertise<std_msgs::String>("reference_link_name", 1, true);
    end_link_name_publisher_=local_nh_.advertise<std_msgs::String>("end_link_name", 1, true);
    elfin_pose = local_nh_.advertise<geometry_msgs::Pose>("elfin_pose",1,true);

    ref_link_name_msg_.data=group_->getPlanningFrame();
    end_link_name_msg_.data=group_->getEndEffectorLink();

    ref_link_name_publisher_.publish(ref_link_name_msg_);
    end_link_name_publisher_.publish(end_link_name_msg_);
    pub_pose_timer=local_nh_.createTimer(ros::Duration(0.005), &ElfinBasicAPI::posePubTimer_cb, this);
    pub_pose_timer.start();
}

ElfinBasicAPI::~ElfinBasicAPI()
{
    if(teleop_api_ != NULL)
        delete teleop_api_;
    if(motion_api_ != NULL)
        delete motion_api_;
}

void ElfinBasicAPI::dynamicReconfigureCallback(ElfinBasicAPIDynamicReconfigureConfig &config, uint32_t level)
{
    setVelocityScaling(config.velocity_scaling);
}

void ElfinBasicAPI::setVelocityScaling(double data)
{
    velocity_scaling_=data;
    teleop_api_->setVelocityScaling(velocity_scaling_);
    motion_api_->setVelocityScaling(velocity_scaling_);
}

void ElfinBasicAPI::posePubTimer_cb(const ros::TimerEvent& evt)
{
    if(tf_listener_.waitForTransform(group_->getPlanningFrame(), group_->getEndEffectorLink(),ros::Time(0), ros::Duration(2))){
        tf_listener_.lookupTransform(group_->getPlanningFrame(), group_->getEndEffectorLink(),ros::Time(0), poseTransForm);
        geometry_msgs::Pose msg;
        msg.position.x = poseTransForm.getOrigin().x();
        msg.position.y = poseTransForm.getOrigin().y();
        msg.position.z = poseTransForm.getOrigin().z();
        msg.orientation.x = poseTransForm.getRotation().x();
        msg.orientation.y = poseTransForm.getRotation().y();
        msg.orientation.z = poseTransForm.getRotation().z();
        msg.orientation.w = poseTransForm.getRotation().w();
        elfin_pose.publish(msg);
    }
}

bool ElfinBasicAPI::setRefLink_cb(elfin_robot_msgs::SetString::Request &req, elfin_robot_msgs::SetString::Response &resp)
{
    if(!tf_listener_.frameExists(req.data))
    {
        resp.success=false;
        std::string result="There is no frame named ";
        result.append(req.data);
        resp.message=result;
        return true;
    }

    teleop_api_->setRefFrames(req.data);
    motion_api_->setRefFrames(req.data);

    ref_link_name_msg_.data=req.data;
    ref_link_name_publisher_.publish(ref_link_name_msg_);

    resp.success=true;
    resp.message="Setting reference link succeed";
    return true;
}

bool ElfinBasicAPI::setEndLink_cb(elfin_robot_msgs::SetString::Request &req, elfin_robot_msgs::SetString::Response &resp)
{
    if(!tf_listener_.frameExists(req.data))
    {
        resp.success=false;
        std::string result="There is no frame named ";
        result.append(req.data);
        resp.message=result;
        return true;
    }

    teleop_api_->setEndFrames(req.data);
    motion_api_->setEndFrames(req.data);

    end_link_name_msg_.data=req.data;
    end_link_name_publisher_.publish(end_link_name_msg_);

    resp.success=true;
    resp.message="Setting end link succeed";
    return true;
}

bool ElfinBasicAPI::enableRobot_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    // Check request
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }

    // Check if there is a real driver
    if(!raw_enable_robot_client_.exists())
    {
        resp.message="there is no real driver running";
        resp.success=false;
        return true;
    }

    std_srvs::SetBool::Request req_tmp;
    std_srvs::SetBool::Response resp_tmp;

    // Stop active controllers
    if(!stopActCtrlrs(resp_tmp))
    {
        resp=resp_tmp;
        return true;
    }

    usleep(500000);

    // Check motion state
    if(!get_motion_state_client_.exists())
    {
        resp.message="there is no get_motion_state service";
        resp.success=false;
        return true;
    }

    req_tmp.data=true;
    get_motion_state_client_.call(req_tmp, resp_tmp);
    if(resp_tmp.success)
    {
        resp.message="failed to enable the robot, it's moving";
        resp.success=false;
        return true;
    }

    // Check position alignment state
    if(!get_pos_align_state_client_.exists())
    {
        resp.message="there is no get_pos_align_state service";
        resp.success=false;
        return true;
    }

    req_tmp.data=true;
    get_pos_align_state_client_.call(req_tmp, resp_tmp);
    if(!resp_tmp.success)
    {
        resp.message="failed to enable the robot, commands aren't aligned with actual positions";
        resp.success=false;
        return true;
    }

    // Check enable service
    if(!raw_enable_robot_client_.exists())
    {
        resp.message="there is no real driver running";
        resp.success=false;
        return true;
    }

    // Enable servos
    raw_enable_robot_request_.data=true;
    raw_enable_robot_client_.call(raw_enable_robot_request_, raw_enable_robot_response_);
    resp=raw_enable_robot_response_;

    // Start default controller
    if(!startElfinCtrlr(resp_tmp))
    {
        resp=resp_tmp;
        return true;
    }

    return true;
}

bool ElfinBasicAPI::disableRobot_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    // Check request
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }

    // Check disable service
    if(!raw_disable_robot_client_.exists())
    {
        resp.message="there is no real driver running";
        resp.success=false;
        return true;
    }

    // Disable servos
    raw_disable_robot_request_.data=true;
    raw_disable_robot_client_.call(raw_disable_robot_request_, raw_disable_robot_response_);
    resp=raw_disable_robot_response_;

    std_srvs::SetBool::Response resp_tmp;

    // Stop active controllers
    if(!stopActCtrlrs(resp_tmp))
    {
        resp=resp_tmp;
        return true;
    }

    return true;
}

bool ElfinBasicAPI::stopActCtrlrs(std_srvs::SetBool::Response &resp)
{
    // Check list controllers service
    if(!list_controllers_client_.exists())
    {
        resp.message="there is no controller manager";
        resp.success=false;
        return false;
    }

    // Find controllers to stop
    controller_manager_msgs::ListControllers::Request list_controllers_request;
    controller_manager_msgs::ListControllers::Response list_controllers_response;
    list_controllers_client_.call(list_controllers_request, list_controllers_response);
    std::vector<std::string> controllers_to_stop;
    controllers_to_stop.clear();

    controller_joint_names_.clear();
    for(int i=0; i<list_controllers_response.controller.size(); i++)
    {
        std::string name_tmp=list_controllers_response.controller[i].name;
        std::vector<controller_manager_msgs::HardwareInterfaceResources> resrc_tmp=list_controllers_response.controller[i].claimed_resources;
        if(strcmp(name_tmp.c_str(), elfin_controller_name_.c_str())==0)
        {
            for(int j=0; j<resrc_tmp.size(); j++)
            {
                controller_joint_names_.insert(controller_joint_names_.end(), resrc_tmp[j].resources.begin(),
                                               resrc_tmp[j].resources.end());
            }
            break;
        }
    }

    for(int i=0; i<list_controllers_response.controller.size(); i++)
    {
        std::string state_tmp=list_controllers_response.controller[i].state;
        std::string name_tmp=list_controllers_response.controller[i].name;
        std::vector<controller_manager_msgs::HardwareInterfaceResources> resrc_tmp=list_controllers_response.controller[i].claimed_resources;
        if(strcmp(state_tmp.c_str(), "running")==0)
        {
            bool break_flag=false;
            for(int j=0; j<resrc_tmp.size(); j++)
            {
                for(int k=0; k<controller_joint_names_.size(); k++)
                {
                    if(std::find(resrc_tmp[j].resources.begin(), resrc_tmp[j].resources.end(),
                                 controller_joint_names_[k])!=resrc_tmp[j].resources.end())
                    {
                        break_flag=true;
                        controllers_to_stop.push_back(name_tmp);
                    }
                    if(break_flag)
                    {
                        break;
                    }
                }
                if(break_flag)
                {
                    break;
                }
            }
        }
    }

    // Stop active controllers
    if(controllers_to_stop.size()>0)
    {
        // Check switch controller service
        if(!switch_controller_client_.exists())
        {
            resp.message="there is no controller manager";
            resp.success=false;
            return false;
        }

        // Stop active controllers
        controller_manager_msgs::SwitchController::Request switch_controller_request;
        controller_manager_msgs::SwitchController::Response switch_controller_response;
        switch_controller_request.start_controllers.clear();
        switch_controller_request.stop_controllers=controllers_to_stop;
        switch_controller_request.strictness=switch_controller_request.STRICT;

        switch_controller_client_.call(switch_controller_request, switch_controller_response);
        if(!switch_controller_response.ok)
        {
            resp.message="Failed to stop active controllers";
            resp.success=false;
            return false;
        }
    }

    return true;
}

bool ElfinBasicAPI::startElfinCtrlr(std_srvs::SetBool::Response &resp)
{
    // Check switch controller service
    if(!switch_controller_client_.exists())
    {
        resp.message="there is no controller manager";
        resp.success=false;
        return false;
    }

    // Start active controllers
    controller_manager_msgs::SwitchController::Request switch_controller_request;
    controller_manager_msgs::SwitchController::Response switch_controller_response;
    switch_controller_request.start_controllers.clear();
    switch_controller_request.start_controllers.push_back(elfin_controller_name_);
    switch_controller_request.stop_controllers.clear();
    switch_controller_request.strictness=switch_controller_request.STRICT;

    switch_controller_client_.call(switch_controller_request, switch_controller_response);
    if(!switch_controller_response.ok)
    {
        resp.message="Failed to start the default controller";
        resp.success=false;
        return false;
    }

    return true;
}

}
