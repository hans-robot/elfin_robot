/*
Created on Tues Oct 17 09:34:50 2017

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

#include "elfin_ethercat_driver/elfin_ethercat_driver.h"

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

    // Initialize count_zeros_
    if(!ed_nh_.hasParam("count_zeros"))
    {
        ROS_ERROR("Please set the param %s/count_zeros", ed_nh_.getNamespace().c_str());
        exit(0);
    }

    ed_nh_.getParam("count_zeros", count_zeros_);

    // Check the number of joint names, reduction ratios and count_zeros
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
    if(count_zeros_.size()!=slave_no_.size()*2)
    {
        ROS_ERROR("the number of count_zeros is %lu, it should be %lu", count_zeros_.size(), slave_no_.size()*2);
        exit(0);
    }

    // Initialize ethercat_client_
    ethercat_clients_.clear();
    ethercat_clients_.resize(slave_no_.size());
    for(int i=0; i<slave_no_.size(); i++)
    {
        ethercat_clients_[i]=new elfin_ethercat_driver::ElfinEtherCATClient(manager, slave_no_[i]);
    }

    //Initialize ros service server
    get_txpdo_server_=ed_nh_.advertiseService("get_txpdo", &ElfinEtherCATDriver::getTxPDO_cb, this);
    get_rxpdo_server_=ed_nh_.advertiseService("get_rxpdo", &ElfinEtherCATDriver::getRxPDO_cb, this);
    get_current_position_server_=ed_nh_.advertiseService("get_current_position", &ElfinEtherCATDriver::getCurrentPosition_cb, this);
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
    if(recognizePosition())
        ROS_INFO("positions are recognized automatically");
    else
        ROS_INFO("positions aren't recognized automatically");
}

ElfinEtherCATDriver::~ElfinEtherCATDriver()
{
    for(int i=0; i<ethercat_clients_.size(); i++)
    {
        if(ethercat_clients_[i]!=NULL)
            delete ethercat_clients_[i];
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

int32_t ElfinEtherCATDriver::getCountZero(size_t n)
{
    return count_zeros_[n];
}

bool ElfinEtherCATDriver::recognizePosition()
{
    std_srvs::SetBool::Request request;
    std_srvs::SetBool::Response response;
    if(getFaultState())
    {
        request.data=true;
        clearFault_cb(request, response);
    }
    else
    {
        response.success=true;
    }

    if(response.success)
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
        ROS_WARN("there are some faults, positions can't be recognized");
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
            flag_tmp=flag_tmp && ethercat_clients_[i]->readInput_unit(elfin_txpdo::AXIS1_STATUSWORD)==0x8237
                    && ethercat_clients_[i]->readInput_unit(elfin_txpdo::AXIS2_STATUSWORD)==0x8237
                    && ethercat_clients_[i]->readInput_unit(elfin_txpdo::UDM_STATUS)==0xffff0000;
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
        resp.message="position recognition failed";
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
//    pthread_t tid;
//    pthread_create(&tid, NULL, update_loop, (void *)&ec);
//    ros::Rate r(10);
//    while (ros::ok()) {
//        ec.readInput();
//        ec.readOutput();
//        ec.pubInput();
//        ec.pubOutput();
//        ros::spinOnce();
//        r.sleep();
//    }
}
