/*
Created on Mon Sep 17 11:15 2018

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
    read_sdo_=io_nh_.advertiseService("read_di", &ElfinEtherCATIOClient::readSDO_cb, this); // 20201117: support for 485 end
    read_do_=io_nh_.advertiseService("read_do", &ElfinEtherCATIOClient::readDO_cb, this); // 20201130: support for read DO
    write_sdo_=io_nh_.advertiseService("write_do",&ElfinEtherCATIOClient::writeSDO_cb, this); // 20201117: support for 485 end
    get_txsdo_server_=io_nh_.advertiseService("get_txpdo", &ElfinEtherCATIOClient::getTxSDO_cb, this); // 20201120: support for 485 end
    get_rxsdo_server_=io_nh_.advertiseService("get_rxpdo", &ElfinEtherCATIOClient::getRxSDO_cb, this); // 20201120: support for 485 end

}

ElfinEtherCATIOClient::~ElfinEtherCATIOClient()
{

}

int16_t ElfinEtherCATIOClient::readInput_unit(int n)
{

    int16_t map;
    map = (manager_->readSDO<int16_t>(4, 0x6001, 0x01));// << 16; // read the end DI
    // printf("map: %d\n", map);
    return map;
}

int32_t ElfinEtherCATIOClient::readOutput_unit(int n)
{
    int32_t map;
    map = (manager_->readSDO<int32_t>(4, 0x7001, 0x01)) << 12; 
    return map;
}

void ElfinEtherCATIOClient::writeOutput_unit(int n, int32_t val)
{

    manager_->writeSDO<int32_t>(4,0x7001,0x01, val >> 12);
}

// 20201116: read the end SDO
int32_t ElfinEtherCATIOClient::readSDO_unit(int n)
{
    if(n<0 || n>=pdo_input.size())
        return 0x0000;
    // 20201116: build the connection.
    manager_->writeSDO<int>(3,0x3100,0x0,1); // Modbus DO command
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x3101,0x0,0x010040); // 64 connect to Modbus
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x3102,0x0,0x010001); // Modbus Addr & count
    usleep(50000);
    // 0x2126, L_4 is end DI, H_4 is button DI.
    int32_t map;
    map = (manager_->readSDO<int32_t>(3, 0x2126, 0x0)) << 16; // read the end DI
    manager_->writeSDO<int>(3,0x3100,0x0,0); // Modbus DO command 0
    usleep(50000);
    return map;
}

// 20201130: read the end DO
int32_t ElfinEtherCATIOClient::readDO_unit(int n)
{
    if(n<0 || n>=pdo_input.size())
        return 0x0000;
    // 20201130: build the connection.
    manager_->writeSDO<int>(3,0x3100,0x0,1); // Modbus DO command
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x3101,0x0,0x010040); // 64 connect to Modbus
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x3102,0x0,0x010001); // Modbus Addr & count
    usleep(50000);
    // 0x310C, DO.
    int32_t map;
    map = (manager_->readSDO<int32_t>(4, 0x7001, 0x0)) << 12; // read the end DO
    manager_->writeSDO<int>(3,0x3100,0x0,0); // Modbus DO command 0
    usleep(50000);
    return map;
}

// 20201117: write the end SDO LED and DO
int32_t ElfinEtherCATIOClient::writeSDO_unit(int32_t val)
{
    // 20201119: high the LED and DO of the end
    manager_->writeSDO<int>(3,0x3100,0x0,1); // Modbus DO command
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x3101,0x0,0x010006); // Modbus SlaveID & Function
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x3102,0x0,0x010001); // Modbus Addr & count
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x310C,0x0, val >> 12); // Write the LED and DO
    usleep(50000);
    manager_->writeSDO<int>(3,0x3100,0x0,0); // Modbus DO command 0
    usleep(50000);

    return 0;
}


// 20201120: add the getTxSDO and getRxSDO for confirming the same as old IO
std::string ElfinEtherCATIOClient::getTxSDO()
{
    int length=20;
    uint8_t map[length];
    char temp[8];
    std::string result="slave";
    result.reserve(160);
    result.append("4_txpdo:\n");
    for (unsigned i = 0; i < length; ++i)
    {
        map[i] = 0x00;
        sprintf(temp,"0x%.2x",(uint8_t)map[i]);
        result.append(temp, 4);
        result.append(":");
    }
    result.append("\n");
    return result;
}

std::string ElfinEtherCATIOClient::getRxSDO()
{
    int length=4;
    uint8_t map[length];
    char temp[8];
    std::string result="slave";
    result.reserve(160);
    result.append("4_rxpdo:\n");
    for (unsigned i = 0; i < length; ++i)
    {
        map[i] = 0x00;
        sprintf(temp,"0x%.2x",(uint8_t)map[i]);
        result.append(temp, 4);
        result.append(":");
    }
    result.append("\n");
    return result;

}

// 20201116: read the end SDO
bool ElfinEtherCATIOClient::readSDO_cb(elfin_robot_msgs::ElfinIODRead::Request &req, elfin_robot_msgs::ElfinIODRead::Response &resp)
{
    resp.digital_input=readInput_unit(elfin_io_txpdo::DIGITAL_INPUT);//elfin_io_txpdo::DIGITAL_INPUT
    return true;
}

// 20201130: read the end DO
bool ElfinEtherCATIOClient::readDO_cb(elfin_robot_msgs::ElfinIODRead::Request &req, elfin_robot_msgs::ElfinIODRead::Response &resp)
{
    resp.digital_input=readOutput_unit(elfin_io_txpdo::DIGITAL_INPUT); // 20201130: digital_input for convenience
    return true;
}

// 20201117: write the end SDO
bool ElfinEtherCATIOClient::writeSDO_cb(elfin_robot_msgs::ElfinIODWrite::Request &req, elfin_robot_msgs::ElfinIODWrite::Response &resp)
{
    writeOutput_unit(elfin_io_rxpdo::DIGITAL_OUTPUT,req.digital_output);
    resp.success=true;
    return true;
}

// 20201120: add the getTxSDO and getRxSDO for confirming the same as old IO
bool ElfinEtherCATIOClient::getRxSDO_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }

    resp.success=true;
    resp.message=getRxSDO();
    return true;
}

bool ElfinEtherCATIOClient::getTxSDO_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }

    resp.success=true;
    resp.message=getTxSDO();
    return true;
}

}
