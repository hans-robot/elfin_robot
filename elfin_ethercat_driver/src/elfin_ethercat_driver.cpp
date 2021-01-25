/*
Created on Mon Sep 17 10:31:06 2018

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

#include <elfin_ethercat_driver/elfin_ethercat_driver.h>

namespace elfin_ethercat_driver {

ElfinEtherCATDriver::ElfinEtherCATDriver(EtherCatManager *manager, std::string driver_name, const ros::NodeHandle &nh):
    driver_name_(driver_name), root_nh_(nh), ed_nh_(nh, driver_name)
{
    // Initialize slave_no_
    int slave_no_array_default[3]={1, 2, 3};
    std::vector<int> slave_no_default;
    slave_no_default.clear();
    slave_no_default.reserve(3);
    for(int i=0; i<3; i++)
    {
        slave_no_default.push_back(slave_no_array_default[i]);
    }
    ed_nh_.param<std::vector<int> >("slave_no", slave_no_, slave_no_default);

    // Initialize joint_names_
    std::vector<std::string> joint_names_default;
    joint_names_default.clear();
    joint_names_default.reserve(2*slave_no_.size());
    for(int i=0; i<slave_no_.size(); i++)
    {
        std::string num_1=boost::lexical_cast<std::string>(2*(i+1)-1);
        std::string num_2=boost::lexical_cast<std::string>(2*(i+1));
        std::string name_1="joint";
        std::string name_2="joint";
        name_1.append(num_1);
        name_2.append(num_2);

        joint_names_default.push_back(name_1);
        joint_names_default.push_back(name_2);
    }
    ed_nh_.param<std::vector<std::string> >("joint_names", joint_names_, joint_names_default);

    // Initialize reduction_ratios_
    std::vector<double> reduction_ratios_default;
    reduction_ratios_default.clear();
    reduction_ratios_default.reserve(2*slave_no_.size());
    for(int i=0; i<slave_no_.size(); i++)
    {
        reduction_ratios_default.push_back(101);
        reduction_ratios_default.push_back(101);
    }
    ed_nh_.param<std::vector<double> >("reduction_ratios", reduction_ratios_, reduction_ratios_default);

    // Check the values of reduction ratios
    for(int i=0; i<reduction_ratios_.size(); i++)
    {
        if(reduction_ratios_[i]<1)
        {
            ROS_ERROR("reduction_ratios[%i] is too small", i);
            exit(0);
        }
    }

    // Initialize axis_position_factors_
    std::vector<double> axis_position_factors_default;
    axis_position_factors_default.clear();
    axis_position_factors_default.reserve(2*slave_no_.size());
    for(int i=0; i<slave_no_.size(); i++)
    {
        axis_position_factors_default.push_back(131072);
        axis_position_factors_default.push_back(131072);
    }
    ed_nh_.param<std::vector<double> >("axis_position_factors", axis_position_factors_, axis_position_factors_default);

    // Check the values of axis position factors
    for(int i=0; i<axis_position_factors_.size(); i++)
    {
        if(axis_position_factors_[i]<1)
        {
            ROS_ERROR("axis_position_factors[%i] is too small", i);
            exit(0);
        }
    }

    // Initialize axis_torque_factors_
    if(!ed_nh_.hasParam("axis_torque_factors"))
    {
        ROS_ERROR("Please set the param %s/axis_torque_factors", ed_nh_.getNamespace().c_str());
        ROS_ERROR("For more details: elfin_robot_bringup/config/elfin_drivers_example.yaml");
        exit(0);
    }

    ed_nh_.getParam("axis_torque_factors", axis_torque_factors_);

    // Check the values of axis torque factors
    for(int i=0; i<axis_torque_factors_.size(); i++)
    {
        if(axis_torque_factors_[i]<1)
        {
            ROS_ERROR("axis_torque_factors[%i] is too small", i);
            exit(0);
        }
    }

    // Initialize count_zeros_
    if(!ed_nh_.hasParam("count_zeros"))
    {
        ROS_ERROR("Please set the param %s/count_zeros", ed_nh_.getNamespace().c_str());
        ROS_ERROR("For more details: elfin_robot_bringup/config/elfin_drivers_example.yaml");
        exit(0);
    }

    ed_nh_.getParam("count_zeros", count_zeros_);

    // Check the number of joint names, reduction ratios, axis position factors
    // axis torque factors and count_zeros
    if(joint_names_.size()!=slave_no_.size()*2)
    {
        ROS_ERROR("the number of joint names is %lu, it should be %lu", joint_names_.size(), slave_no_.size()*2);
        exit(0);
    }
    if(reduction_ratios_.size()!=slave_no_.size()*2)
    {
        ROS_ERROR("the number of reduction ratios is %lu, it should be %lu", reduction_ratios_.size(), slave_no_.size()*2);
        exit(0);
    }
    if(axis_position_factors_.size()!=slave_no_.size()*2)
    {
        ROS_ERROR("the number of axis position factors is %lu, it should be %lu", axis_position_factors_.size(), slave_no_.size()*2);
        exit(0);
    }
    if(axis_torque_factors_.size()!=slave_no_.size()*2)
    {
        ROS_ERROR("the number of axis torque factors is %lu, it should be %lu", axis_torque_factors_.size(), slave_no_.size()*2);
        exit(0);
    }
    if(count_zeros_.size()!=slave_no_.size()*2)
    {
        ROS_ERROR("the number of count_zeros is %lu, it should be %lu", count_zeros_.size(), slave_no_.size()*2);
        exit(0);
    }

    // Initialize count_rad_factors
    count_rad_factors_.resize(count_zeros_.size());
    for(int i=0; i<count_rad_factors_.size(); i++)
    {
        count_rad_factors_[i]=reduction_ratios_[i]*axis_position_factors_[i]/(2*M_PI);
    }

    // Initialize motion_threshold_ and pos_align_threshold_
    motion_threshold_=5e-5;
    pos_align_threshold_=5e-5;

    // Initialize ethercat_client_
    ethercat_clients_.clear();
    ethercat_clients_.resize(slave_no_.size());
    for(int i=0; i<slave_no_.size(); i++)
    {
        ethercat_clients_[i]=new ElfinEtherCATClient(manager, slave_no_[i]);
    }

    // Initialize io_slave_no_
    int io_slave_no_array_default[1]={4};
    std::vector<int> io_slave_no_default;
    io_slave_no_default.clear();
    io_slave_no_default.reserve(1);
    for(int i=0; i<1; i++)
    {
        io_slave_no_default.push_back(io_slave_no_array_default[i]);
    }
    ed_nh_.param<std::vector<int> >("io_slave_no", io_slave_no_, io_slave_no_default);

    // Initialize ethercat_io_client_
    ethercat_io_clients_.clear();
    ethercat_io_clients_.resize(io_slave_no_.size());
    std::string io_port="io_port";
    for(int i=0; i<io_slave_no_.size(); i++)
    {
        std::string num=boost::lexical_cast<std::string>(i+1);
        std::string io_port_name=io_port.append(num);
        ethercat_io_clients_[i]=new ElfinEtherCATIOClient(manager, io_slave_no_[i], ed_nh_, io_port_name);
    }

    //Initialize ros service server
    get_txpdo_server_=ed_nh_.advertiseService("get_txpdo", &ElfinEtherCATDriver::getTxPDO_cb, this);
    get_rxpdo_server_=ed_nh_.advertiseService("get_rxpdo", &ElfinEtherCATDriver::getRxPDO_cb, this);
    get_current_position_server_=ed_nh_.advertiseService("get_current_position", &ElfinEtherCATDriver::getCurrentPosition_cb, this);
    get_motion_state_server_=ed_nh_.advertiseService("get_motion_state", &ElfinEtherCATDriver::getMotionState_cb, this);
    get_pos_align_state_server_=ed_nh_.advertiseService("get_pos_align_state", &ElfinEtherCATDriver::getPosAlignState_cb, this);
    enable_robot_=ed_nh_.advertiseService("enable_robot", &ElfinEtherCATDriver::enableRobot_cb, this);
    disable_robot_=ed_nh_.advertiseService("disable_robot", &ElfinEtherCATDriver::disableRobot_cb, this);
    clear_fault_=ed_nh_.advertiseService("clear_fault", &ElfinEtherCATDriver::clearFault_cb, this);
    recognize_position_=ed_nh_.advertiseService("recognize_position", &ElfinEtherCATDriver::recognizePosition_cb, this);

    // Initialize ros publisher
    enable_state_pub_=ed_nh_.advertise<std_msgs::Bool>("enable_state", 1);
    fault_state_pub_=ed_nh_.advertise<std_msgs::Bool>("fault_state", 1);

    // Initialize ros timer
    status_update_period_.sec=0;
    status_update_period_.nsec=1e+8;
    status_timer_=ed_nh_.createTimer(status_update_period_, &ElfinEtherCATDriver::updateStatus, this);
    status_timer_.start();

    // Recognize the Positions
    bool recognize_flag;
    ed_nh_.param<bool>("automatic_recognition", recognize_flag, true);
    if(recognize_flag)
    {
        printf("\n recognizing joint positions, please wait a few minutes ... ... \n");
        if(recognizePosition())
            ROS_INFO("positions are recognized automatically");
        else
            ROS_INFO("positions aren't recognized automatically");
    }
}

ElfinEtherCATDriver::~ElfinEtherCATDriver()
{
    for(int i=0; i<ethercat_clients_.size(); i++)
    {
        if(ethercat_clients_[i]!=NULL)
            delete ethercat_clients_[i];
    }
    for(int i=0; i<ethercat_io_clients_.size(); i++)
    {
        if(ethercat_io_clients_[i]!=NULL)
            delete ethercat_io_clients_[i];
    }
}

// true: enabled; false: disabled
bool ElfinEtherCATDriver::getEnableState()
{
    bool enable_flag_tmp=true;
    for(int i=0; i<ethercat_clients_.size(); i++)
    {
        enable_flag_tmp=enable_flag_tmp && ethercat_clients_[i]->isEnabled();
    }
    return enable_flag_tmp;
}

// true: there is a fault; false: there is no fault
bool ElfinEtherCATDriver::getFaultState()
{
    bool fault_flag_tmp=false;
    for(int i=0; i<ethercat_clients_.size(); i++)
    {
        fault_flag_tmp=fault_flag_tmp || ethercat_clients_[i]->isWarning();
    }
    return fault_flag_tmp;
}

// true: robot is moving; false: robot is not moving
bool ElfinEtherCATDriver::getMotionState()
{
    std::vector<double> previous_pos;
    std::vector<double> last_pos;

    previous_pos.resize(count_zeros_.size());
    last_pos.resize(count_zeros_.size());

    int32_t count1, count2;
    for(int i=0; i<ethercat_clients_.size(); i++)
    {
        ethercat_clients_[i]->getActPosCounts(count1, count2);
        previous_pos[2*i]=count1/count_rad_factors_[2*i];
        previous_pos[2*i+1]=count2/count_rad_factors_[2*i+1];
    }

    usleep(10000);

    for(int i=0; i<ethercat_clients_.size(); i++)
    {
        ethercat_clients_[i]->getActPosCounts(count1, count2);
        last_pos[2*i]=count1/count_rad_factors_[2*i];
        last_pos[2*i+1]=count2/count_rad_factors_[2*i+1];
    }

    for(int i=0; i<previous_pos.size(); i++)
    {
        if(fabs(last_pos[i]-previous_pos[i])>motion_threshold_)
        {
            return true;
        }
    }

    return false;
}

// true: command position counts are aligned with actual position counts
// false: command position counts aren't aligned with actual position counts
bool ElfinEtherCATDriver::getPosAlignState()
{
    std::vector<double> act_pos;
    std::vector<double> cmd_pos;

    act_pos.resize(count_zeros_.size());
    cmd_pos.resize(count_zeros_.size());

    int32_t count1, count2;
    for(int i=0; i<ethercat_clients_.size(); i++)
    {
        ethercat_clients_[i]->getActPosCounts(count1, count2);
        act_pos[2*i]=count1/count_rad_factors_[2*i];
        act_pos[2*i+1]=count2/count_rad_factors_[2*i+1];

        ethercat_clients_[i]->getCmdPosCounts(count1, count2);
        cmd_pos[2*i]=count1/count_rad_factors_[2*i];
        cmd_pos[2*i+1]=count2/count_rad_factors_[2*i+1];
    }

    for(int i=0; i<act_pos.size(); i++)
    {
        if(fabs(cmd_pos[i]-act_pos[i])>pos_align_threshold_)
        {
            return false;
        }
    }

    return true;
}

void ElfinEtherCATDriver::updateStatus(const ros::TimerEvent &te)
{
    enable_state_msg_.data=getEnableState();
    fault_state_msg_.data=getFaultState();

    enable_state_pub_.publish(enable_state_msg_);
    fault_state_pub_.publish(fault_state_msg_);
}

size_t ElfinEtherCATDriver::getEtherCATClientNumber()
{
    return ethercat_clients_.size();
}

ElfinEtherCATClient* ElfinEtherCATDriver::getEtherCATClientPtr(size_t n)
{
    return ethercat_clients_[n];
}

std::string ElfinEtherCATDriver::getJointName(size_t n)
{
    return joint_names_[n];
}

double ElfinEtherCATDriver::getReductionRatio(size_t n)
{
    return reduction_ratios_[n];
}

double ElfinEtherCATDriver::getAxisPositionFactor(size_t n)
{
    return axis_position_factors_[n];
}

double ElfinEtherCATDriver::getAxisTorqueFactor(size_t n)
{
    return axis_torque_factors_[n];
}

int32_t ElfinEtherCATDriver::getCountZero(size_t n)
{
    return count_zeros_[n];
}

bool ElfinEtherCATDriver::recognizePosition()
{
    std_srvs::SetBool::Request request;
    request.data=true;
    std_srvs::SetBool::Response response_clear_fault;
    std_srvs::SetBool::Response response_disable;
    if(getFaultState())
    {
        clearFault_cb(request, response_clear_fault);
    }
    else
    {
        response_clear_fault.success=true;
    }

    disableRobot_cb(request, response_disable);

    if(response_clear_fault.success && response_disable.success)
    {
        std::vector<pthread_t> tids;
        tids.resize(ethercat_clients_.size());

        std::vector<int> threads;
        threads.resize(ethercat_clients_.size());

        for(int i=0; i<ethercat_clients_.size(); i++)
        {
            threads[i]=pthread_create(&tids[i], NULL, ethercat_clients_[i]->recognizePoseCmd, (void *)ethercat_clients_[i]);
        }
        for(int i=0; i<ethercat_clients_.size(); i++)
        {
            pthread_join(tids[i], NULL);
        }
    }
    else
    {
        ROS_WARN("there are some faults or the motors aren't disabled, positions can't be recognized");
        return false;
    }
    return true;
}

bool ElfinEtherCATDriver::getTxPDO_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }
    if(ethercat_clients_.size()==0)
    {
        resp.success=false;
        resp.message="there is no ethercat client";
        return true;
    }

    std::string result=ethercat_clients_[0]->getTxPDO();
    unsigned int reference_length=result.size();
    result.reserve(ethercat_clients_.size() * reference_length);

    for(int i=1; i<ethercat_clients_.size(); i++)
    {
        result.append(ethercat_clients_[i]->getTxPDO());
    }

    resp.success=true;
    resp.message=result;
    return true;
}

bool ElfinEtherCATDriver::getRxPDO_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }
    if(ethercat_clients_.size()==0)
    {
        resp.success=false;
        resp.message="there is no ethercat client";
        return true;
    }

    std::string result=ethercat_clients_[0]->getRxPDO();
    unsigned int reference_length=result.size();
    result.reserve(ethercat_clients_.size() * reference_length);

    for(int i=1; i<ethercat_clients_.size(); i++)
    {
        result.append(ethercat_clients_[i]->getRxPDO());
    }

    resp.success=true;
    resp.message=result;
    return true;
}

bool ElfinEtherCATDriver::getCurrentPosition_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }
    if(ethercat_clients_.size()==0)
    {
        resp.success=false;
        resp.message="there is no ethercat client";
        return true;
    }

    std::string result=ethercat_clients_[0]->getCurrentPosition();

    for(int i=1; i<ethercat_clients_.size(); i++)
    {
        result.append(ethercat_clients_[i]->getCurrentPosition());
    }

    resp.success=true;
    resp.message=result;
    return true;
}

bool ElfinEtherCATDriver::getMotionState_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="request's data is false";
        return true;
    }
    resp.message="true: robot is moving; false: robot is not moving";
    resp.success=getMotionState();
    return true;
}

bool ElfinEtherCATDriver::getPosAlignState_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="request's data is false";
        return true;
    }
    resp.message="true: cmd pos aligned with actual pos; false: cmd pos not aligned with actual pos";
    resp.success=getPosAlignState();
    return true;
}

bool ElfinEtherCATDriver::enableRobot_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }
    if(ethercat_clients_.size()==0)
    {
        resp.success=false;
        resp.message="there is no ethercat client";
        return true;
    }
    if(getFaultState())
    {
        resp.success=false;
        resp.message="please clear fault first";
        return true;
    }

    std::vector<pthread_t> tids;
    tids.resize(ethercat_clients_.size());

    std::vector<int> threads;
    threads.resize(ethercat_clients_.size());

    for(int i=0; i<ethercat_clients_.size(); i++)
    {
        threads[i]=pthread_create(&tids[i], NULL, ethercat_clients_[i]->setEnable, (void *)ethercat_clients_[i]);
    }
    for(int i=0; i<ethercat_clients_.size(); i++)
    {
        pthread_join(tids[i], NULL);
    }

    struct timespec before, tick;
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    bool flag_tmp;
    while (ros::ok())
    {
        flag_tmp=true;
        for(int i=0; i<ethercat_clients_.size(); i++)
        {
            flag_tmp=flag_tmp && ethercat_clients_[i]->isEnabled();
        }
        if(flag_tmp)
        {
            resp.success=true;
            resp.message="robot is enabled";
            return true;
        }
        if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 20e+9)
        {
            resp.success=false;
            resp.message="robot is not enabled";
            return true;
        }
        usleep(100000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }
}

bool ElfinEtherCATDriver::disableRobot_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }
    if(ethercat_clients_.size()==0)
    {
        resp.success=false;
        resp.message="there is no ethercat client";
        return true;
    }

    std::vector<pthread_t> tids;
    tids.resize(ethercat_clients_.size());

    std::vector<int> threads;
    threads.resize(ethercat_clients_.size());

    for(int i=0; i<ethercat_clients_.size(); i++)
    {
        threads[i]=pthread_create(&tids[i], NULL, ethercat_clients_[i]->setDisable, (void *)ethercat_clients_[i]);
    }
    for(int i=0; i<ethercat_clients_.size(); i++)
    {
        pthread_join(tids[i], NULL);
    }

    struct timespec before, tick;
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    bool flag_tmp;
    while (ros::ok())
    {
        flag_tmp=false;
        for(int i=0; i<ethercat_clients_.size(); i++)
        {
            flag_tmp=flag_tmp || ethercat_clients_[i]->isEnabled();
        }
        if(!flag_tmp)
        {
            resp.success=true;
            resp.message="robot is disabled";
            return true;
        }
        if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 20e+9)
        {
            resp.success=false;
            resp.message="robot is not disabled";
            return true;
        }
        usleep(100000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }
}

bool ElfinEtherCATDriver::clearFault_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }
    if(ethercat_clients_.size()==0)
    {
        resp.success=false;
        resp.message="there is no ethercat client";
        return true;
    }

    for(int i=0; i<ethercat_clients_.size(); i++)
    {
        ethercat_clients_[i]->resetFault();
    }

    struct timespec before, tick;
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    bool flag_tmp;
    while (ros::ok())
    {
        flag_tmp=false;
        for(int i=0; i<ethercat_clients_.size(); i++)
        {
            flag_tmp=flag_tmp || ethercat_clients_[i]->isWarning();
        }
        if(!flag_tmp)
        {
            resp.success=true;
            resp.message="Faults are cleared";
            return true;
        }
        if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 10e+9)
        {
            resp.success=false;
            resp.message="There are still Faults";
            return true;
        }
        usleep(100000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }
}

bool ElfinEtherCATDriver::recognizePosition_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }
    if(ethercat_clients_.size()==0)
    {
        resp.success=false;
        resp.message="there is no ethercat client";
        return true;
    }

    bool flag_tmp=recognizePosition();

    if(flag_tmp)
    {
        resp.success=true;
        resp.message="positions are recognized";
        return true;
    }
    else
    {
        resp.success=false;
        resp.message="position recognition failed, please disable the servos first";
        return true;
    }
}

int32_t ElfinEtherCATDriver::getIntFromStr(std::string str)
{
    if(str.size()>8 || str.size()%2 !=0)
    {
        ROS_ERROR("%s 's length should be an even number and less than 8", str.c_str());
        exit(0);
    }

    unsigned char map[4];
    int j=0;

    for(int i=0; i<str.size(); i+=2)
    {
        unsigned char high=str[str.size()-2-i];
        unsigned char low=str[str.size()-1-i];

        if(high>='0' && high<='9')
            high = high-'0';
        else if(high>='A' && high<='F')
            high = high - 'A' + 10;
        else if(high>='a' && high<='f')
            high = high - 'a' + 10;
        else
        {
            ROS_ERROR("%s 's not a hex number", str.c_str());
            exit(0);
        }

        if(low>='0' && low<='9')
            low = low-'0';
        else if(low>='A' && low<='F')
            low = low - 'A' + 10;
        else if(low>='a' && low<='f')
            low = low - 'a' + 10;
        else
        {
            ROS_ERROR("%s 's not a hex number", str.c_str());
            exit(0);
        }

        map[j++]=high << 4 | low;
    }
    for(int k=j; k<4; k++)
    {
        unsigned char tmp=map[k];
        map[k]=tmp-tmp;
    }

    int32_t result;
    result=*(int32_t *)(map);
    return result;
}

} // end namespace

int main(int argc, char** argv)
{
    ros::init(argc,argv,"elfin_ethercat_driver", ros::init_options::AnonymousName);
    elfin_ethercat_driver::EtherCatManager em("eth0");
    elfin_ethercat_driver::ElfinEtherCATDriver ed(&em, "elfin");

    ros::spin();
}
