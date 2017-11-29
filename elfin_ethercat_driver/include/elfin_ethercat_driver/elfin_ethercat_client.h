/*
Created on Mon Oct 16 10:52:07 2017

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

#ifndef ELFIN_ETHERCAT_CLIENT_H
#define ELFIN_ETHERCAT_CLIENT_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <vector>
#include "elfin_ethercat_driver/elfin_ethercat_manager.h"

#include <pthread.h>
#include <time.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace elfin_txpdo
{
    const int AXIS1_STATUSWORD=0;
    const int AXIS1_ACTPOSITION=1;
    const int AXIS1_ACTCUR=2;
    const int AXIS1_ERRORCODE=3;

    const int AXIS2_STATUSWORD=4;
    const int AXIS2_ACTPOSITION=5;
    const int AXIS2_ACTCUR=6;
    const int AXIS2_ERRORCODE=7;

    const int UDM_STATUS=8;
    const int ACCELERATION_X=9;
    const int ACCELERATION_Y=10;
    const int ACCELERATION_Z=11;
}

namespace elfin_rxpdo
{
    const int AXIS1_CONTROLWORD=0;
    const int AXIS1_TARGET_POSITION=1;
    const int AXIS1_ENDATPOS_FLASH=2;
    const int AXIS1_FEEDFORWARD_CUR=3;

    const int AXIS2_CONTROLWORD=4;
    const int AXIS2_TARGET_POSITION=5;
    const int AXIS2_ENDATPOS_FLASH=6;
    const int AXIS2_FEEDFORWARD_CUR=7;

    const int UDM_CMD=8;
}


namespace elfin_ethercat_driver
{
typedef struct {
    std::string name;
    int32_t value;
    uint8_t channel;
}ElfinPDOunit;

class ElfinEtherCATClient
{
private:
    EtherCatManager* manager_;
    ros::NodeHandle n_;
    ros::Publisher pub_input_;
    ros::Publisher pub_output_;
    ros::ServiceServer server_enable_;
    ros::ServiceServer server_reset_fault_;
    std_msgs::String txpdo_msg_;
    std_msgs::String rxpdo_msg_;
    std::vector<ElfinPDOunit> pdo_input; // txpdo
    std::vector<ElfinPDOunit> pdo_output; //rxpdo
    int slave_no_;

public:
    ElfinEtherCATClient(EtherCatManager* manager, int slave_no);
    int32_t readInput_unit(int n);
    int32_t readOutput_unit(int n);
    void writeOutput_unit(int n, int32_t val);
    void readInput();
    void readOutput();
    void writeOutput();
    std::string getTxPDO();
    std::string getRxPDO();
    std::string getCurrentPosition();
    void pubInput();
    void pubOutput();
    bool isEnabled();
    static void *setEnable(void *threadarg);
    static void *setDisable(void *threadarg);
    bool isWarning();
    void resetFault();
    bool enable_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    bool reset_fault_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
};
}
#endif
