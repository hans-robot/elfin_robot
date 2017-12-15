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

ElfinBasicAPI::ElfinBasicAPI(moveit::planning_interface::MoveGroup *group, std::string action_name):
    group_(group), action_client_(action_name, true), local_nh_("~")
{
    teleop_api_=new ElfinTeleopAPI(group, action_name);
    motion_api_=new ElfinMotionAPI(group, action_name);

    set_velocity_scaling_server_=local_nh_.advertiseService("set_velocity_scaling", &ElfinBasicAPI::setVelocityScaling_cb, this);
    update_velocity_scaling_server_=local_nh_.advertiseService("update_velocity_scaling", &ElfinBasicAPI::updateVelocityScaling_cb, this);

    gui_velocity_scaling_update_pub_=local_nh_.advertise<std_msgs::Empty>("gui_velocity_scaling_update", 1);

    dynamic_reconfigure_server_.setCallback(boost::bind(&ElfinBasicAPI::dynamicReconfigureCallback, this, _1, _2));
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
}

bool ElfinBasicAPI::setVelocityScaling_cb(elfin_robot_msgs::SetFloat64::Request &req, elfin_robot_msgs::SetFloat64::Response &resp)
{
    if(req.data<0.01 || req.data>1)
    {
        resp.success=false;
        resp.message="the data of the request should be in range [0.01, 1.0]";
        return true;
    }
    setVelocityScaling(req.data);
    local_nh_.setParam("velocity_scaling", req.data);
    gui_velocity_scaling_update_pub_.publish(empty_msg_);
    resp.success=true;
    resp.message="set velocity scaling successfully";
    return true;
}

bool ElfinBasicAPI::updateVelocityScaling_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    local_nh_.setParam("velocity_scaling", velocity_scaling_);
    resp.success=true;
    resp.message="the ros parameter "+local_nh_.getNamespace()+"/velocity_scaling is updated";
    return true;
}

}
