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
    write_do_=io_nh_.advertiseService("write_do", &ElfinEtherCATIOClient::writeDO_cb, this);
    read_di_=io_nh_.advertiseService("read_di", &ElfinEtherCATIOClient::readDI_cb, this);
    get_txpdo_server_=io_nh_.advertiseService("get_txpdo", &ElfinEtherCATIOClient::getTxPDO_cb, this);
    get_rxpdo_server_=io_nh_.advertiseService("get_rxpdo", &ElfinEtherCATIOClient::getRxPDO_cb, this);

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

std::string ElfinEtherCATIOClient::getTxPDO()
{
    int length=20;
    uint8_t map[length];
    char temp[8];
    std::string result="slave";
    result.reserve(160); // the size of result is actually 115
    std::string slave_num=boost::lexical_cast<std::string>(slave_no_);
    result.append(slave_num);
    result.append("_txpdo:\n");
    for (unsigned i = 0; i < length; ++i)
    {
        map[i] = manager_->readInput(slave_no_, i);
        sprintf(temp,"0x%.2x",(uint8_t)map[i]);
        result.append(temp, 4);
        result.append(":");
    }
    result.append("\n");
    return result;
}

std::string ElfinEtherCATIOClient::getRxPDO()
{
    int length=4;
    uint8_t map[length];
    char temp[8];
    std::string result="slave";
    result.reserve(160); // the size of result is actually 35
    std::string slave_num=boost::lexical_cast<std::string>(slave_no_);
    result.append(slave_num);
    result.append("_rxpdo:\n");
    for (unsigned i = 0; i < length; ++i)
    {
        map[i] = manager_->readOutput(slave_no_, i);
        sprintf(temp,"0x%.2x",(uint8_t)map[i]);
        result.append(temp, 4);
        result.append(":");
    }
    result.append("\n");
    return result;
}

bool ElfinEtherCATIOClient::writeDO_cb(elfin_robot_msgs::ElfinIODWrite::Request &req, elfin_robot_msgs::ElfinIODWrite::Response &resp)
{
    writeOutput_unit(elfin_io_rxpdo::DIGITAL_OUTPUT, req.digital_output);
    resp.success=true;
    return true;
}

bool ElfinEtherCATIOClient::readDI_cb(elfin_robot_msgs::ElfinIODRead::Request &req, elfin_robot_msgs::ElfinIODRead::Response &resp)
{
    resp.digital_input=readInput_unit(elfin_io_txpdo::DIGITAL_INPUT);
    return true;
}

bool ElfinEtherCATIOClient::getTxPDO_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }

    resp.success=true;
    resp.message=getTxPDO();
    return true;
}

bool ElfinEtherCATIOClient::getRxPDO_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }

    resp.success=true;
    resp.message=getRxPDO();
    return true;
}

}
