/*
Created on Tue Jan 16 11:34 2018

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

#ifndef ELFIN_ETHERCAT_IO_CLIENT_H
#define ELFIN_ETHERCAT_IO_CLIENT_H

#include <ros/ros.h>
#include <vector>
#include <elfin_ethercat_driver/elfin_ethercat_manager.h>
#include <elfin_robot_msgs/ElfinIODRead.h>
#include <elfin_robot_msgs/ElfinIODWrite.h>
#include <std_srvs/SetBool.h>

#include <pthread.h>
#include <time.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace elfin_io_txpdo {

const int DIGITAL_INPUT=0;
const int ANALOG_INPUT_CHANNEL1=1;
const int ANALOG_INPUT_CHANNEL2=2;
const int SMART_CAMERA_X=3;
const int SMART_CAMERA_Y=4;

}

namespace elfin_io_rxpdo {

const int DIGITAL_OUTPUT=0;

}

namespace  elfin_ethercat_driver {

class ElfinEtherCATIOClient{

private:
    EtherCatManager* manager_;
    ros::NodeHandle n_;
    ros::NodeHandle io_nh_;
    std::vector<ElfinPDOunit> pdo_input; // txpdo
    std::vector<ElfinPDOunit> pdo_output; //rxpdo
    int slave_no_;

    ros::ServiceServer write_do_;
    ros::ServiceServer read_di_;
    ros::ServiceServer get_txpdo_server_;
    ros::ServiceServer get_rxpdo_server_;

public:
    ElfinEtherCATIOClient(EtherCatManager* manager, int slave_no, const ros::NodeHandle& nh, std::string io_port_name);
    ~ElfinEtherCATIOClient();
    int32_t readInput_unit(int n);
    int32_t readOutput_unit(int n);
    void writeOutput_unit(int n, int32_t val);

    std::string getTxPDO();
    std::string getRxPDO();

    bool writeDO_cb(elfin_robot_msgs::ElfinIODWrite::Request &req, elfin_robot_msgs::ElfinIODWrite::Response &resp);
    bool readDI_cb(elfin_robot_msgs::ElfinIODRead::Request &req, elfin_robot_msgs::ElfinIODRead::Response &resp);
    bool getTxPDO_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    bool getRxPDO_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

};

}

#endif
