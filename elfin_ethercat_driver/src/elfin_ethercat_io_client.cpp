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

#include <elfin_ethercat_driver/elfin_ethercat_io_client.h>

namespace elfin_ethercat_driver {

ElfinEtherCATIOClient::ElfinEtherCATIOClient(EtherCatManager *manager, int slave_no, const ros::NodeHandle &nh, std::string io_port_name):
    manager_(manager), slave_no_(slave_no), io_nh_(nh, io_port_name)
{
    // init pdo_input and output
    std::string name_pdo_input[5]={"Digital_Input", "Analog_Input_channel1", "Analog_Input_channel2",
                                  "Smart_Camera_X", "Smart_Camera_Y"};
    uint8_t channel_pdo_input[5]={0, 4, 8, 12, 16};
    pdo_input.clear();
    ElfinPDOunit unit_tmp;
    for(unsigned i=0; i<5; ++i)
    {
        unit_tmp.name=name_pdo_input[i];
        unit_tmp.channel=channel_pdo_input[i];
        pdo_input.push_back(unit_tmp);
    }

    std::string name_pdo_output[1]={"Digital_Output"};
    uint8_t channel_pdo_output[1]={0};
    pdo_output.clear();
    for(unsigned i=0; i<1; ++i)
    {
        unit_tmp.name=name_pdo_output[i];
        unit_tmp.channel=channel_pdo_output[i];
        pdo_output.push_back(unit_tmp);
    }

    // Initialize services
    io_server_=io_nh_.advertiseService("io_service", &ElfinEtherCATIOClient::ioService_cb, this);

}

ElfinEtherCATIOClient::~ElfinEtherCATIOClient()
{

}

int32_t ElfinEtherCATIOClient::readInput_unit(int n)
{
    if(n<0 || n>=pdo_input.size())
        return 0x0000;
    uint8_t map[4];
    for(int i=0; i<4; i++)
    {
        map[i]=manager_->readInput(slave_no_, pdo_input[n].channel+i);
    }
    int32_t value_tmp=*(int32_t *)(map);
    return value_tmp;
}

int32_t ElfinEtherCATIOClient::readOutput_unit(int n)
{
    if(n<0 || n>=pdo_output.size())
        return 0x0000;
    uint8_t map[4];
    for(int i=0; i<4; i++)
    {
        map[i]=manager_->readOutput(slave_no_, pdo_output[n].channel+i);
    }
    int32_t value_tmp=*(int32_t *)(map);
    return value_tmp;
}

void ElfinEtherCATIOClient::writeOutput_unit(int n, int32_t val)
{
    if(n<0 || n>=pdo_output.size())
        return;
    uint8_t map_tmp;
    for(int i=0; i<4; i++)
    {
        map_tmp=(val>>8*i) & 0x00ff;
        manager_->write(slave_no_, pdo_output[n].channel+i, map_tmp);
    }
}

bool ElfinEtherCATIOClient::ioService_cb(elfin_robot_msgs::ElfinIO::Request &req, elfin_robot_msgs::ElfinIO::Response &resp)
{
    writeOutput_unit(elfin_io_rxpdo::DIGITAL_OUTPUT, req.digital_output);
    usleep(10000);
    resp.digital_input=readInput_unit(elfin_io_txpdo::DIGITAL_INPUT);
    resp.analog_input_channel1=readInput_unit(elfin_io_txpdo::ANALOG_INPUT_CHANNEL1);
    resp.analog_input_channel2=readInput_unit(elfin_io_txpdo::ANALOG_INPUT_CHANNEL2);
    resp.smart_camera_x=readInput_unit(elfin_io_txpdo::SMART_CAMERA_X);
    resp.smart_camera_y=readInput_unit(elfin_io_txpdo::SMART_CAMERA_Y);

    return true;
}

}
