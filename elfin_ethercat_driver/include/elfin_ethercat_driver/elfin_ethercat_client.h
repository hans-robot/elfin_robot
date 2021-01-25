/*
Created on Mon Sep 17 09:42:14 2018

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

#ifndef ELFIN_ETHERCAT_CLIENT_H
#define ELFIN_ETHERCAT_CLIENT_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <vector>
#include <elfin_ethercat_driver/elfin_ethercat_manager.h>

#include <pthread.h>
#include <time.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace elfin_txpdo
{
    const int AXIS1_STATUSWORD_L16=0;
    const int AXIS1_ACTTORQUE_H16=0;
    const int AXIS1_ACTPOSITION=1;
    const int AXIS1_ACTVELOCITY_L16=2;
    const int AXIS1_ERRORCODE_L16=3;
    const int AXIS1_MODES_OF_OPERATION_DISPLAY_BYTE2=3;

    const int AXIS2_STATUSWORD_L16=4;
    const int AXIS2_ACTTORQUE_H16=4;
    const int AXIS2_ACTPOSITION=5;
    const int AXIS2_ACTVELOCITY_L16=6;
    const int AXIS2_ERRORCODE_L16=7;
    const int AXIS2_MODES_OF_OPERATION_DISPLAY_BYTE2=7;
}

namespace elfin_rxpdo
{
    const int AXIS1_CONTROLWORD_L16=0;
    const int AXIS1_MODES_OF_OPERATION_BYTE2=0;
    const int AXIS1_TARGET_POSITION=1;
    const int AXIS1_TARGET_TORQUE_L16=2;
    const int AXIS1_VELFF_H16=2;

    const int AXIS2_CONTROLWORD_L16=3;
    const int AXIS2_MODES_OF_OPERATION_BYTE2=3;
    const int AXIS2_TARGET_POSITION=4;
    const int AXIS2_TARGET_TORQUE_L16=5;
    const int AXIS2_VELFF_H16=5;
}


namespace elfin_ethercat_driver
{
class ElfinEtherCATClient
{
private:
    EtherCatManager* manager_; // old namespace
    ros::NodeHandle n_;
    ros::Publisher pub_input_;
    ros::Publisher pub_output_;
    ros::ServiceServer server_enable_;
    ros::ServiceServer server_reset_fault_;
    ros::ServiceServer server_open_brake_;
    ros::ServiceServer server_close_brake_;
    std_msgs::String txpdo_msg_;
    std_msgs::String rxpdo_msg_;
    std::vector<ElfinPDOunit> pdo_input; // txpdo old namespace
    std::vector<ElfinPDOunit> pdo_output; //rxpdo old namespace
    int slave_no_;

public:
    ElfinEtherCATClient(EtherCatManager* manager, int slave_no); // old namespace
    ~ElfinEtherCATClient();
    int32_t readInput_unit(int n);
    int32_t readOutput_unit(int n);
    void writeOutput_unit(int n, int32_t val);
    int16_t readInput_half_unit(int n, bool high_16);
    int16_t readOutput_half_unit(int n, bool high_16);
    void writeOutput_half_unit(int n, int16_t val, bool high_16);
    int8_t readInput_unit_byte(int n, bool high_16, bool high_8);
    int8_t readOutput_unit_byte(int n, bool high_16, bool high_8);
    void writeOutput_unit_byte(int n, int8_t val, bool high_16, bool high_8);
    int32_t getAxis1PosCnt();
    int32_t getAxis2PosCnt();
    void setAxis1PosCnt(int32_t pos_cnt);
    void setAxis2PosCnt(int32_t pos_cnt);
    int16_t getAxis1VelCnt();
    int16_t getAxis2VelCnt();
    void setAxis1VelFFCnt(int16_t vff_cnt);
    void setAxis2VelFFCnt(int16_t vff_cnt);
    int16_t getAxis1TrqCnt();
    int16_t getAxis2TrqCnt();
    void setAxis1TrqCnt(int16_t trq_cnt);
    void setAxis2TrqCnt(int16_t trq_cnt);
    void readInput();
    void readOutput();
    void writeOutput();
    std::string getTxPDO();
    std::string getRxPDO();
    std::string getCurrentPosition();
    void getActPosCounts(int32_t &pos_act_count_1, int32_t &pos_act_count_2);
    void getCmdPosCounts(int32_t &pos_cmd_count_1, int32_t &pos_cmd_count_2);
    void pubInput();
    void pubOutput();
    void clearPoseFault();
    bool recognizePose();
    bool isEnabled();
    static void *setEnable(void *threadarg);
    static void *setDisable(void *threadarg);
    static void *recognizePoseCmd(void *threadarg);
    bool isWarning();
    void resetFault();
    bool inPosMode();
    bool inTrqMode();
    bool inPosBasedMode();
    void setPosMode();
    void setTrqMode();
    bool enable_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    bool reset_fault_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    bool open_brake_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    bool close_brake_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
};
}
#endif
