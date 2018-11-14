/*
Created on Mon Oct 16 10:45:10 2017

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2017 - 2018, Han's Robot Co., Ltd.
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

#include <elfin_ethercat_driver/elfin_ethercat_client.h>

namespace elfin_ethercat_driver
{
ElfinEtherCATClient::ElfinEtherCATClient(EtherCatManager *manager, int slave_no):
    manager_(manager), slave_no_(slave_no)
{
    std::string info_tx_name="elfin_module_info_tx_slave";
    std::string info_rx_name="elfin_module_info_rx_slave";
    std::string enable_server_name="elfin_module_enable_slave";
    std::string reset_fault_server_name="elfin_module_reset_fault_slave";
    std::string open_brake_server_name="elfin_module_open_brake_slave";
    std::string close_brake_server_name="elfin_module_close_brake_slave";

    std::string slave_num=boost::lexical_cast<std::string>(slave_no);

    info_tx_name.append(slave_num);
    info_rx_name.append(slave_num);
    enable_server_name.append(slave_num);
    reset_fault_server_name.append(slave_num);
    open_brake_server_name.append(slave_num);
    close_brake_server_name.append(slave_num);

    pub_input_=n_.advertise<std_msgs::String>(info_tx_name, 1);
    pub_output_=n_.advertise<std_msgs::String>(info_rx_name, 1);
    server_enable_=n_.advertiseService(enable_server_name, &ElfinEtherCATClient::enable_cb, this);
    server_reset_fault_=n_.advertiseService(reset_fault_server_name, &ElfinEtherCATClient::reset_fault_cb, this);
    server_open_brake_=n_.advertiseService(open_brake_server_name, &ElfinEtherCATClient::open_brake_cb, this);
    server_close_brake_=n_.advertiseService(close_brake_server_name, &ElfinEtherCATClient::close_brake_cb, this);

    // init pdo_input and output
    std::string name_pdo_input[12]={"Axis1_Statusword", "Axis1_ActPosition", "Axis1_ActCur", "Axis1_ErrorCode",
                                   "Axis2_Statusword", "Axis2_ActPosition", "Axis2_ActCur", "Axis2_ErrorCode",
                                   "UDM_Status", "Acceleration_X", "Acceleration_Y", "Acceleration_Z"};
    uint8_t channel_pdo_input[12]={0, 4, 8, 12, 40, 44, 48, 52, 80, 84, 88, 92};
    pdo_input.clear();
    ElfinPDOunit unit_tmp;
    for(unsigned i=0; i<12; ++i)
    {
        unit_tmp.name=name_pdo_input[i];
        unit_tmp.channel=channel_pdo_input[i];
        pdo_input.push_back(unit_tmp);
    }

    std::string name_pdo_output[9]={"Axis1_Controlword", "Axis1_Target_position", "Axis1_EndatPos_Flash",
                                    "Axis1_FeedForWard_Cur", "Axis2_Controlword", "Axis2_Target_position",
                                    "Axis2_EndatPos_Flash", "Axis2_FeedForWard_Cur", "UDM_Cmd"};
    uint8_t channel_pdo_output[9]={0, 4, 8, 12, 40, 44, 48, 52, 80};
    pdo_output.clear();
    for(unsigned i=0; i<9; ++i)
    {
        unit_tmp.name=name_pdo_output[i];
        unit_tmp.channel=channel_pdo_output[i];
        pdo_output.push_back(unit_tmp);
    }
}

ElfinEtherCATClient::~ElfinEtherCATClient()
{

}

int32_t ElfinEtherCATClient::readInput_unit(int n)
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

int32_t ElfinEtherCATClient::readOutput_unit(int n)
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

void ElfinEtherCATClient::writeOutput_unit(int n, int32_t val)
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

int16_t ElfinEtherCATClient::readInput_half_unit(int n, bool high_16)
{
    if(n<0 || n>=pdo_input.size())
        return 0x0000;
    int offset;
    if(high_16)
        offset=2;
    else
        offset=0;
    uint8_t map[2];
    for(int i=0; i<2; i++)
    {
        map[i]=manager_->readInput(slave_no_, pdo_input[n].channel+offset+i);
    }
    int16_t value_tmp=*(int16_t *)(map);
    return value_tmp;
}

int16_t ElfinEtherCATClient::readOutput_half_unit(int n, bool high_16)
{
    if(n<0 || n>=pdo_output.size())
        return 0x0000;
    int offset;
    if(high_16)
        offset=2;
    else
        offset=0;
    uint8_t map[2];
    for(int i=0; i<2; i++)
    {
        map[i]=manager_->readOutput(slave_no_, pdo_output[n].channel+offset+i);
    }
    int16_t value_tmp=*(int16_t *)(map);
    return value_tmp;
}

void ElfinEtherCATClient::writeOutput_half_unit(int n, int16_t val, bool high_16)
{
    if(n<0 || n>=pdo_output.size())
        return;
    int offset;
    if(high_16)
        offset=2;
    else
        offset=0;
    uint8_t map_tmp;
    for(int i=0; i<2; i++)
    {
        map_tmp=(val>>8*i) & 0x00ff;
        manager_->write(slave_no_, pdo_output[n].channel+offset+i, map_tmp);
    }
}

int8_t ElfinEtherCATClient::readInput_unit_byte(int n, bool high_16, bool high_8)
{
    if(n<0 || n>=pdo_input.size())
        return 0x0000;
    int offset;
    if(high_16)
    {
        if(high_8)
            offset=3;
        else
            offset=2;
    }
    else
    {
        if(high_8)
            offset=1;
        else
            offset=0;
    }
    uint8_t map[1];
    for(int i=0; i<1; i++)
    {
        map[i]=manager_->readInput(slave_no_, pdo_input[n].channel+offset+i);
    }
    int8_t value_tmp=*(int8_t *)(map);
    return value_tmp;
}

int8_t ElfinEtherCATClient::readOutput_unit_byte(int n, bool high_16, bool high_8)
{
    if(n<0 || n>=pdo_output.size())
        return 0x0000;
    int offset;
    if(high_16)
    {
        if(high_8)
            offset=3;
        else
            offset=2;
    }
    else
    {
        if(high_8)
            offset=1;
        else
            offset=0;
    }
    uint8_t map[1];
    for(int i=0; i<1; i++)
    {
        map[i]=manager_->readOutput(slave_no_, pdo_output[n].channel+offset+i);
    }
    int8_t value_tmp=*(int8_t *)(map);
    return value_tmp;
}

void ElfinEtherCATClient::writeOutput_unit_byte(int n, int8_t val, bool high_16, bool high_8)
{
    if(n<0 || n>=pdo_output.size())
        return;
    int offset;
    if(high_16)
    {
        if(high_8)
            offset=3;
        else
            offset=2;
    }
    else
    {
        if(high_8)
            offset=1;
        else
            offset=0;
    }
    uint8_t map_tmp;
    for(int i=0; i<1; i++)
    {
        map_tmp=(val>>8*i) & 0x00ff;
        manager_->write(slave_no_, pdo_output[n].channel+offset+i, map_tmp);
    }
}

int32_t ElfinEtherCATClient::getAxis1PosCnt()
{
    return readInput_unit(elfin_txpdo::AXIS1_ACTPOSITION);
}

int32_t ElfinEtherCATClient::getAxis2PosCnt()
{
    return readInput_unit(elfin_txpdo::AXIS2_ACTPOSITION);
}

void ElfinEtherCATClient::setAxis1PosCnt(int32_t pos_cnt)
{
    writeOutput_unit(elfin_rxpdo::AXIS1_TARGET_POSITION, pos_cnt);
}

void ElfinEtherCATClient::setAxis2PosCnt(int32_t pos_cnt)
{
    writeOutput_unit(elfin_rxpdo::AXIS2_TARGET_POSITION, pos_cnt);
}

int16_t ElfinEtherCATClient::getAxis1TrqCnt()
{
    return readInput_half_unit(elfin_txpdo::AXIS1_ACTCUR_L16, false);
}

int16_t ElfinEtherCATClient::getAxis2TrqCnt()
{
    return readInput_half_unit(elfin_txpdo::AXIS2_ACTCUR_L16, false);
}

void ElfinEtherCATClient::setAxis1TrqCnt(int16_t trq_cnt)
{
    writeOutput_half_unit(elfin_rxpdo::AXIS1_FEEDFORWARD_CUR_L16, trq_cnt, false);
}

void ElfinEtherCATClient::setAxis2TrqCnt(int16_t trq_cnt)
{
    writeOutput_half_unit(elfin_rxpdo::AXIS2_FEEDFORWARD_CUR_L16, trq_cnt, false);
}

void ElfinEtherCATClient::readInput()
{
    for(int i=0; i<pdo_input.size(); i++)
    {
        readInput_unit(i);
    }
}

void ElfinEtherCATClient::readOutput()
{
    for(int i=0; i<pdo_output.size(); i++)
    {
        readOutput_unit(i);
    }
}

std::string ElfinEtherCATClient::getTxPDO()
{
    int length=96;
    uint8_t map[length];
    char temp[8];
    std::string result="slave";
    result.reserve(640); // the size of result is actually 495
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

std::string ElfinEtherCATClient::getRxPDO()
{
    int length=84;
    uint8_t map[length];
    char temp[8];
    std::string result="slave";
    result.reserve(640); // the size of result is actually 435
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

std::string ElfinEtherCATClient::getCurrentPosition()
{
    std::string result="slave";
    std::string slave_num=boost::lexical_cast<std::string>(slave_no_);
    result.append(slave_num);
    result.append("_current_position:\n");
    int32_t current_position_1=readInput_unit(elfin_txpdo::AXIS1_ACTPOSITION);
    std::string tmp_str_1=boost::lexical_cast<std::string>(current_position_1);
    result.append("axis1: ");
    result.append(tmp_str_1);
    result.append(", ");
    int32_t current_position_2=readInput_unit(elfin_txpdo::AXIS2_ACTPOSITION);
    std::string tmp_str_2=boost::lexical_cast<std::string>(current_position_2);
    result.append("axis2: ");
    result.append(tmp_str_2);
    result.append(". \n");
    return result;
}

void ElfinEtherCATClient::getActPosCounts(int32_t &pos_act_count_1, int32_t &pos_act_count_2)
{
    pos_act_count_1=readInput_unit(elfin_txpdo::AXIS1_ACTPOSITION);
    pos_act_count_2=readInput_unit(elfin_txpdo::AXIS2_ACTPOSITION);
}

void ElfinEtherCATClient::getCmdPosCounts(int32_t &pos_cmd_count_1, int32_t &pos_cmd_count_2)
{
    pos_cmd_count_1=readOutput_unit(elfin_rxpdo::AXIS1_TARGET_POSITION);
    pos_cmd_count_2=readOutput_unit(elfin_rxpdo::AXIS2_TARGET_POSITION);
}

void ElfinEtherCATClient::pubInput()
{
    txpdo_msg_.data=getTxPDO();
    pub_input_.publish(txpdo_msg_);
    txpdo_msg_.data.clear();
}

void ElfinEtherCATClient::pubOutput()
{
    rxpdo_msg_.data=getRxPDO();
    pub_output_.publish(rxpdo_msg_);
    rxpdo_msg_.data.clear();
}

void ElfinEtherCATClient::clearPoseFault()
{
    // channel1
    writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x011f);
    writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x001f);
    writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x0800);
    usleep(100000);
    // channel2
    writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x001f);
    writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x011f);
    writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x0800);
    usleep(100000);
}

bool ElfinEtherCATClient::recognizePose()
{
    if(readInput_unit(elfin_txpdo::UDM_STATUS) == 0x11110000)
    {
        //channel1
        writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x201f);
        writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x001f);
        writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x0300);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(readInput_unit(elfin_txpdo::UDM_STATUS) == 0xffff0000)
            {
                writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x001f);
                writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x001f);
                writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x0000);
                usleep(100000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 20e+9)
            {
                ROS_WARN("recognizePose failed while pose recognition in slave %i, channel 1", slave_no_);
                return false;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
    }
    else
    {
        ROS_WARN("recognizePose failed in slave %i, channel 1, the reason might be there is a fault or the motor is enabled", slave_no_);
        return false;
    }

    if(readInput_unit(elfin_txpdo::UDM_STATUS) == 0x11110000)
    {
        //channel2
        writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x001f);
        writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x201f);
        writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x3000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(readInput_unit(elfin_txpdo::UDM_STATUS) == 0xffff0000)
            {
                writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x001f);
                writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x001f);
                writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x0000);
                usleep(100000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 20e+9)
            {
                ROS_WARN("recognizePose failed while pose recognition in slave %i, channel 2", slave_no_);
                return false;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
    }
    else
    {
        ROS_WARN("recognizePose failed in slave %i, channel 2, the reason might be there is a fault or the motor is enabled", slave_no_);
        return false;
    }
    return true;
}

bool ElfinEtherCATClient::isEnabled()
{
    if(inPosBasedMode() || inTrqMode())
        return true;
    else
        return false;
}

void *ElfinEtherCATClient::setEnable(void* threadarg)
{
    ElfinEtherCATClient *pthis=(ElfinEtherCATClient *)threadarg;
    if(pthis->readInput_unit(elfin_txpdo::UDM_STATUS) == 0x6666)
    {
        pthis->clearPoseFault();
        if(!pthis->recognizePose())
            return (void *)0;
    }

    // enable
    if(pthis->readInput_unit(elfin_txpdo::UDM_STATUS) == 0x11110000)
    {
        pthis->writeOutput_unit(elfin_rxpdo::AXIS1_TARGET_POSITION, pthis->readInput_unit(elfin_txpdo::AXIS1_ACTPOSITION));
        pthis->writeOutput_unit(elfin_rxpdo::AXIS2_TARGET_POSITION, pthis->readInput_unit(elfin_txpdo::AXIS2_ACTPOSITION));
        usleep(100000);

        pthis->writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x801f);
        pthis->writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x801f);
        pthis->writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x0044);
        usleep(100000);
    }
    else
    {
        ROS_WARN("UDM status is not 0x11110000 in slave %i, the reason might be there is a fault or the motor is enabled", pthis->slave_no_);
        return (void *)0;
    }
}

void *ElfinEtherCATClient::setDisable(void *threadarg)
{
    ElfinEtherCATClient *pthis=(ElfinEtherCATClient *)threadarg;
    if(pthis->isEnabled())
    {
        pthis->writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x7);
        pthis->writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x7);
        return (void *)0;
    }
    else
    {
        return (void *)0;
    }
}

void *ElfinEtherCATClient::recognizePoseCmd(void *threadarg)
{
    ElfinEtherCATClient *pthis=(ElfinEtherCATClient *)threadarg;

    if(pthis->readInput_unit(elfin_txpdo::UDM_STATUS) == 0x6666)
    {
        pthis->clearPoseFault();
    }

    if(!pthis->recognizePose())
    {
            return (void *)0;
    }

}

bool ElfinEtherCATClient::isWarning()
{
    if((readInput_unit(elfin_txpdo::AXIS1_STATUSWORD) & 0x08)==0x08 || (readInput_unit(elfin_txpdo::AXIS2_STATUSWORD) & 0x08)==0x08 )
        return true;
    else
        return false;
}

void ElfinEtherCATClient::resetFault()
{
    writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x87);
    writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x87);
    usleep(10000);

    writeOutput_unit(elfin_rxpdo::AXIS1_TARGET_POSITION, readInput_unit(elfin_txpdo::AXIS1_ACTPOSITION));
    writeOutput_unit(elfin_rxpdo::AXIS2_TARGET_POSITION, readInput_unit(elfin_txpdo::AXIS2_ACTPOSITION));
    usleep(10000);

    writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x07);
    writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x07);
}

bool ElfinEtherCATClient::inPosMode()
{
    if(readInput_unit(elfin_txpdo::AXIS1_STATUSWORD)==0x8237
       && readInput_unit(elfin_txpdo::AXIS2_STATUSWORD==0x8237))
        return true;
    else
        return false;
}

bool ElfinEtherCATClient::inTrqMode()
{
    if(readInput_unit(elfin_txpdo::AXIS1_STATUSWORD)==0x0a37
       && readInput_unit(elfin_txpdo::AXIS2_STATUSWORD==0x0a37))
        return true;
    else
        return false;
}

bool ElfinEtherCATClient::inPosBasedMode()
{
    return inPosMode();
}

void ElfinEtherCATClient::setPosMode()
{
    writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x801f);
    writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x801f);
    writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x44);
}

void ElfinEtherCATClient::setPosTrqMode()
{
    writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x801f);
    writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x801f);
    writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x144);
}

void ElfinEtherCATClient::setTrqMode()
{
    writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x081f);
    writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x081f);
    writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x66);
}

bool ElfinEtherCATClient::enable_cb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }
    setEnable((void *)this);

    struct timespec before, tick;
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while (ros::ok())
    {
        if(readInput_unit(elfin_txpdo::AXIS1_STATUSWORD)==0x8237 && readInput_unit(elfin_txpdo::AXIS2_STATUSWORD)==0x8237 && readInput_unit(elfin_txpdo::UDM_STATUS)==0xffff0000)
        {
            resp.success=true;
            resp.message="elfin module is enabled";
            return true;
        }
        if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 20e+9)
        {
            resp.success=false;
            resp.message="elfin module is not enabled";
            return true;
        }
        usleep(100000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }
}

bool ElfinEtherCATClient::reset_fault_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="require's data is false";
        return true;
    }

    resetFault();

    resp.success=true;
    resp.message="fault is reset";
    return true;
}

bool ElfinEtherCATClient::open_brake_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="request's data is false";
        return true;
    }

    if(readInput_unit(elfin_txpdo::UDM_STATUS) == 0x11110000)
    {
        //channel1
        writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x401f);
        writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x001f);
        writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x0500);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(readInput_unit(elfin_txpdo::UDM_STATUS) == 0xffff0000)
            {
                writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x001f);
                writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x001f);
                writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x0000);
                usleep(100000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp.message="Channel 1 failed";
                resp.success=false;
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
    }
    else
    {
        resp.message="Channel 1 failed, the reason might be there is a fault or the motor is enabled";
        resp.success=false;
        return true;
    }

    if(readInput_unit(elfin_txpdo::UDM_STATUS) == 0x11110000)
    {
        //channel2
        writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x001f);
        writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x401f);
        writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x5000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(readInput_unit(elfin_txpdo::UDM_STATUS) == 0xffff0000)
            {
                writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x001f);
                writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x001f);
                writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x0000);
                usleep(100000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp.message="Channel 2 failed";
                resp.success=false;
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
    }
    else
    {
        resp.message="Channel 2 failed, the reason might be there is a fault or the motor is enabled";
        resp.success=false;
        return true;
    }

    resp.message="band-type brake is opened";
    resp.success=true;
    return true;
}

bool ElfinEtherCATClient::close_brake_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if(!req.data)
    {
        resp.success=false;
        resp.message="request's data is false";
        return true;
    }

    if(readInput_unit(elfin_txpdo::UDM_STATUS) == 0x11110000)
    {
        //channel1
        writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x401f);
        writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x001f);
        writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x0600);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(readInput_unit(elfin_txpdo::UDM_STATUS) == 0xffff0000)
            {
                writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x001f);
                writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x001f);
                writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x0000);
                usleep(100000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp.message="Channel 1 failed";
                resp.success=false;
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
    }
    else
    {
        resp.message="Channel 1 failed, the reason might be there is a fault or the motor is enabled";
        resp.success=false;
        return true;
    }

    if(readInput_unit(elfin_txpdo::UDM_STATUS) == 0x11110000)
    {
        //channel2
        writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x001f);
        writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x401f);
        writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x6000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(readInput_unit(elfin_txpdo::UDM_STATUS) == 0xffff0000)
            {
                writeOutput_unit(elfin_rxpdo::AXIS1_CONTROLWORD, 0x001f);
                writeOutput_unit(elfin_rxpdo::AXIS2_CONTROLWORD, 0x001f);
                writeOutput_unit(elfin_rxpdo::UDM_CMD, 0x0000);
                usleep(100000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp.message="Channel 2 failed";
                resp.success=false;
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
    }
    else
    {
        resp.message="Channel 2 failed, the reason might be there is a fault or the motor is enabled";
        resp.success=false;
        return true;
    }

    resp.message="band-type brake is closed";
    resp.success=true;
    return true;
}

}

