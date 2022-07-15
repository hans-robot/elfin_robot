/*
Created on Mon Sep 17 10:02:30 2018

@author: Cong Liu, Burb

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
// author: Cong Liu, Burb

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
    std::string name_pdo_input[8]={"Axis1_Statusword and Axis1_Torque_Actual_Value", "Axis1_Position_Actual_Value",
                                   "Axis1_Velocity_Actual_Value", "Axis1_ErrorCode and Axis1_Modes_of_operation_display",
                                   "Axis2_Statusword and Axis2_Torque_Actual_Value", "Axis2_Position_Actual_Value",
                                   "Axis2_Velocity_Actual_Value", "Axis2_ErrorCode and Axis2_Modes_of_operation_display"};
    uint8_t channel_pdo_input[8]={0, 4, 8, 12, 32, 36, 40, 44};
    pdo_input.clear();
    ElfinPDOunit unit_tmp; // old namespace
    for(unsigned i=0; i<8; ++i)
    {
        unit_tmp.name=name_pdo_input[i];
        unit_tmp.channel=channel_pdo_input[i];
        pdo_input.push_back(unit_tmp);
    }

    std::string name_pdo_output[6]={"Axis1_Controlword and Axis1_Modes_of_operation", "Axis1_Target_position", "Axis1_Target_Torque and Axis1_VelFF",
                                    "Axis2_Controlword and Axis2_Modes_of_operation", "Axis2_Target_position", "Axis2_Target_Torque and Axis2_VelFF"};
    uint8_t channel_pdo_output[6]={0, 4, 8, 32, 36, 40};
    pdo_output.clear();
    for(unsigned i=0; i<6; ++i)
    {
        unit_tmp.name=name_pdo_output[i];
        unit_tmp.channel=channel_pdo_output[i];
        pdo_output.push_back(unit_tmp);
    }

    manager_->writeSDO<int8_t>(slave_no, 0x1c12, 0x00, 0x00);
    manager_->writeSDO<int16_t>(slave_no, 0x1c12, 0x01, 0x1600);
    manager_->writeSDO<int16_t>(slave_no, 0x1c12, 0x02, 0x1610);
    manager_->writeSDO<int8_t>(slave_no, 0x1c12, 0x00, 0x02);

    manager_->writeSDO<int8_t>(slave_no, 0x1c13, 0x00, 0x00);
    manager_->writeSDO<int16_t>(slave_no, 0x1c13, 0x01, 0x1a00);
    manager_->writeSDO<int16_t>(slave_no, 0x1c13, 0x02, 0x1a10);
    manager_->writeSDO<int8_t>(slave_no, 0x1c13, 0x00, 0x02);

    writeOutput_half_unit(elfin_rxpdo::AXIS1_CONTROLWORD_L16, 0x0, false);
    writeOutput_half_unit(elfin_rxpdo::AXIS2_CONTROLWORD_L16, 0x0, false);
    writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
    writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
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

int16_t ElfinEtherCATClient::getAxis1VelCnt()
{
    return readInput_half_unit(elfin_txpdo::AXIS1_ACTVELOCITY_L16, false);
}

int16_t ElfinEtherCATClient::getAxis2VelCnt()
{
    return readInput_half_unit(elfin_txpdo::AXIS2_ACTVELOCITY_L16, false);
}

void ElfinEtherCATClient::setAxis1VelFFCnt(int16_t vff_cnt)
{
    writeOutput_half_unit(elfin_rxpdo::AXIS1_VELFF_H16, vff_cnt, true);
}

void ElfinEtherCATClient::setAxis2VelFFCnt(int16_t vff_cnt)
{
    writeOutput_half_unit(elfin_rxpdo::AXIS2_VELFF_H16, vff_cnt, true);
}

int16_t ElfinEtherCATClient::getAxis1TrqCnt()
{
    return readInput_half_unit(elfin_txpdo::AXIS1_ACTTORQUE_H16, true);
}

int16_t ElfinEtherCATClient::getAxis2TrqCnt()
{
    return readInput_half_unit(elfin_txpdo::AXIS2_ACTTORQUE_H16, true);
}

void ElfinEtherCATClient::setAxis1TrqCnt(int16_t trq_cnt)
{
    writeOutput_half_unit(elfin_rxpdo::AXIS1_TARGET_TORQUE_L16, trq_cnt, false);
}

void ElfinEtherCATClient::setAxis2TrqCnt(int16_t trq_cnt)
{
    writeOutput_half_unit(elfin_rxpdo::AXIS2_TARGET_TORQUE_L16, trq_cnt, false);
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
    int length=64;
    uint8_t map[length];
    char temp[8];
    std::string result="slave";
    result.reserve(640); // the size of result is actually 410
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
    int length=64;
    uint8_t map[length];
    char temp[8];
    std::string result="slave";
    result.reserve(640); // the size of result is actually 410
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
    writeOutput_half_unit(elfin_rxpdo::AXIS1_CONTROLWORD_L16, 0x86, false);
    usleep(20000);
    writeOutput_half_unit(elfin_rxpdo::AXIS1_CONTROLWORD_L16, 0x6, false);
    usleep(20000);

    // channel2
    writeOutput_half_unit(elfin_rxpdo::AXIS2_CONTROLWORD_L16, 0x86, false);
    usleep(20000);
    writeOutput_half_unit(elfin_rxpdo::AXIS2_CONTROLWORD_L16, 0x6, false);
    usleep(20000);
}

bool ElfinEtherCATClient::recognizePose()
{
    //channel1
    if((readInput_half_unit(elfin_txpdo::AXIS1_STATUSWORD_L16, false) & 0xc) == 0)
    {
        manager_->writeSDO<int8_t>(slave_no_, 0x6060, 0x0, 0xc);
        writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0xc, true, false);
        usleep(20000);
        manager_->writeSDO<int32_t>(slave_no_, 0x3024, 0x0, 0x11000000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2023, 0x0)==0x200000
               && manager_->readSDO<int32_t>(slave_no_, 0x2024, 0x0)==0x200000)
            {
                manager_->writeSDO<int32_t>(slave_no_, 0x3024, 0x0, 0x33000000);
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 20e+9)
            {
                double result = (manager_->readSDO<int32_t>(slave_no_, 0x2043, 0x0));
                result = result/4096/2.7*49.7*3.3;
                fprintf(stderr,"The voltage of slave %i is: %fV.\n",slave_no_, result);
                ROS_WARN("recognizePose phase1 failed while pose recognition in slave %i, channel 1", slave_no_);
                writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return false;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2023, 0x0)==0
               && manager_->readSDO<int32_t>(slave_no_, 0x2024, 0x0)==0)
            {
                manager_->writeSDO<int32_t>(slave_no_, 0x3024, 0x0, 0x0);
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 5e+9)
            {
                ROS_WARN("recognizePose phase 2 failed while pose recognition in slave %i, channel 1", slave_no_);
                writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return false;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        manager_->writeSDO<int8_t>(slave_no_, 0x6060, 0x0, 0x8);
        writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        usleep(50000);
    }
    else
    {
        ROS_WARN("recognizePose failed in slave %i, channel 1, the reason might be there is a fault or the motor is enabled", slave_no_);
        return false;
    }

    //channel2
    if((readInput_half_unit(elfin_txpdo::AXIS2_STATUSWORD_L16, false) & 0xc) == 0)
    {
        manager_->writeSDO<int8_t>(slave_no_, 0x6860, 0x0, 0xc);
        writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0xc, true, false);
        usleep(20000);
        manager_->writeSDO<int32_t>(slave_no_, 0x3034, 0x0, 0x11000000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2033, 0x0)==0x200000
               && manager_->readSDO<int32_t>(slave_no_, 0x2034, 0x0)==0x200000)
            {
                manager_->writeSDO<int32_t>(slave_no_, 0x3034, 0x0, 0x33000000);
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 20e+9)
            {
                ROS_WARN("recognizePose phase1 failed while pose recognition in slave %i, channel 2", slave_no_);
                writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return false;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2033, 0x0)==0
               && manager_->readSDO<int32_t>(slave_no_, 0x2034, 0x0)==0)
            {
                manager_->writeSDO<int32_t>(slave_no_, 0x3034, 0x0, 0x0);
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 5e+9)
            {
                ROS_WARN("recognizePose phase 2 failed while pose recognition in slave %i, channel 2", slave_no_);
                writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return false;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        manager_->writeSDO<int8_t>(slave_no_, 0x6860, 0x0, 0x8);
        writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        usleep(50000);
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
    if((readInput_half_unit(elfin_txpdo::AXIS1_STATUSWORD_L16, false) & 0xf)==0x7
            && (readInput_half_unit(elfin_txpdo::AXIS2_STATUSWORD_L16, false) & 0xf)==0x7)
        return true;
    else
        return false;
}

void *ElfinEtherCATClient::setEnable(void* threadarg)
{
    ElfinEtherCATClient *pthis=(ElfinEtherCATClient *)threadarg;

    if(pthis->readInput_half_unit(elfin_txpdo::AXIS1_ERRORCODE_L16, false)==0x2000
       || pthis->readInput_half_unit(elfin_txpdo::AXIS2_ERRORCODE_L16, false)==0x2000)
    {
        if(pthis->isWarning())
        {
            pthis->clearPoseFault();
        }

        if(!pthis->recognizePose())
        {
            return (void *)0;
        }
    }

    if(pthis->isWarning())
    {
        pthis->clearPoseFault();
    }

    // enable
    if(pthis->isWarning() || pthis->isEnabled())
    {
        ROS_WARN("setEnable in slave %i failed, the reason might be there is a fault or the motor is enabled", pthis->slave_no_);
        return (void *)0;
    }

    pthis->writeOutput_unit(elfin_rxpdo::AXIS1_TARGET_POSITION, pthis->readInput_unit(elfin_txpdo::AXIS1_ACTPOSITION));
    pthis->writeOutput_unit(elfin_rxpdo::AXIS2_TARGET_POSITION, pthis->readInput_unit(elfin_txpdo::AXIS2_ACTPOSITION));
    usleep(100000);

    pthis->writeOutput_half_unit(elfin_rxpdo::AXIS1_CONTROLWORD_L16, 0x6, false);
    pthis->writeOutput_half_unit(elfin_rxpdo::AXIS2_CONTROLWORD_L16, 0x6, false);

    pthis->writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
    pthis->writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);

    struct timespec before, tick;
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while(ros::ok())
    {
        if(pthis->readInput_half_unit(elfin_txpdo::AXIS1_STATUSWORD_L16, false)==0x21
           && pthis->readInput_half_unit(elfin_txpdo::AXIS2_STATUSWORD_L16, false)==0x21)
        {
            break;
        }
        if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
        {
            ROS_WARN("setEnable phase1 in slave %i failed", pthis->slave_no_);
            return (void *)0;
        }
        usleep(10000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }

    pthis->writeOutput_half_unit(elfin_rxpdo::AXIS1_CONTROLWORD_L16, 0x7, false);
    pthis->writeOutput_half_unit(elfin_rxpdo::AXIS2_CONTROLWORD_L16, 0x7, false);

    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while(ros::ok())
    {
        if(pthis->readInput_half_unit(elfin_txpdo::AXIS1_STATUSWORD_L16, false)==0x23
           && pthis->readInput_half_unit(elfin_txpdo::AXIS2_STATUSWORD_L16, false)==0x23)
        {
            break;
        }
        if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
        {
            ROS_WARN("setEnable phase2 in slave %i failed", pthis->slave_no_);
            return (void *)0;
        }
        usleep(10000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }

    pthis->writeOutput_half_unit(elfin_rxpdo::AXIS1_CONTROLWORD_L16, 0xf, false);
    pthis->writeOutput_half_unit(elfin_rxpdo::AXIS2_CONTROLWORD_L16, 0xf, false);

    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while(ros::ok())
    {
        if(pthis->readInput_half_unit(elfin_txpdo::AXIS1_STATUSWORD_L16, false)==0x27
           && pthis->readInput_half_unit(elfin_txpdo::AXIS2_STATUSWORD_L16, false)==0x27)
        {
            break;
        }
        if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
        {
            ROS_WARN("setEnable phase3 in slave %i failed", pthis->slave_no_);
            return (void *)0;
        }
        usleep(10000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }

    pthis->writeOutput_half_unit(elfin_rxpdo::AXIS1_CONTROLWORD_L16, 0x1f, false);
    pthis->writeOutput_half_unit(elfin_rxpdo::AXIS2_CONTROLWORD_L16, 0x1f, false);

    usleep(100000);
}

void *ElfinEtherCATClient::setDisable(void *threadarg)
{
    ElfinEtherCATClient *pthis=(ElfinEtherCATClient *)threadarg;
    if(pthis->isEnabled())
    {
        pthis->writeOutput_half_unit(elfin_rxpdo::AXIS1_CONTROLWORD_L16, 0x6, false);
        pthis->writeOutput_half_unit(elfin_rxpdo::AXIS2_CONTROLWORD_L16, 0x6, false);
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

    if(pthis->isWarning())
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
    if((readInput_half_unit(elfin_txpdo::AXIS1_STATUSWORD_L16, false) & 0x08)==0x08
       || (readInput_half_unit(elfin_txpdo::AXIS2_STATUSWORD_L16, false) & 0x08)==0x08)
        return true;
    else
        return false;
}

void ElfinEtherCATClient::resetFault()
{
    clearPoseFault();
}

bool ElfinEtherCATClient::inPosMode()
{
    if(isEnabled()
       && readInput_unit_byte(elfin_txpdo::AXIS1_MODES_OF_OPERATION_DISPLAY_BYTE2, true, false)==0x8
       && readInput_unit_byte(elfin_txpdo::AXIS2_MODES_OF_OPERATION_DISPLAY_BYTE2, true, false)==0x8)
        return true;
    else
        return false;
}

bool ElfinEtherCATClient::inTrqMode()
{
    if(isEnabled()
       && readInput_unit_byte(elfin_txpdo::AXIS1_MODES_OF_OPERATION_DISPLAY_BYTE2, true, false)==0xa
       && readInput_unit_byte(elfin_txpdo::AXIS2_MODES_OF_OPERATION_DISPLAY_BYTE2, true, false)==0xa)
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
    writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
    writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
}

void ElfinEtherCATClient::setTrqMode()
{
    writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0xa, true, false);
    writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0xa, true, false);
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
        if(isEnabled())
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

    //channel1
    if((readInput_half_unit(elfin_txpdo::AXIS1_STATUSWORD_L16, false) & 0xc) == 0)
    {
        manager_->writeSDO<int8_t>(slave_no_, 0x6060, 0x0, 0xb);
        writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0xb, true, false);
        usleep(20000);
        manager_->writeSDO<int32_t>(slave_no_, 0x3023, 0x0, 0x11000000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2023, 0x0)==0x300000
               && manager_->readSDO<int32_t>(slave_no_, 0x2024, 0x0)==0x300000)
            {
                usleep(20000);
                manager_->writeSDO<int32_t>(slave_no_, 0x3023, 0x0, 0x33000000);
                usleep(30000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp.message="Channel 1 phase 1 failed";
                resp.success=false;
                writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2023, 0x0)==0
               && manager_->readSDO<int32_t>(slave_no_, 0x2024, 0x0)==0)
            {
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp.message="Channel 1 phase 2 failed";
                resp.success=false;
                writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
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
        writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        return true;
    }

    //channel2
    if((readInput_half_unit(elfin_txpdo::AXIS2_STATUSWORD_L16, false) & 0xc) == 0)
    {
        manager_->writeSDO<int8_t>(slave_no_, 0x6860, 0x0, 0xb);
        writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0xb, true, false);
        usleep(20000);
        manager_->writeSDO<int32_t>(slave_no_, 0x3033, 0x0, 0x11000000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2033, 0x0)==0x300000
               && manager_->readSDO<int32_t>(slave_no_, 0x2034, 0x0)==0x300000)
            {
                usleep(20000);
                manager_->writeSDO<int32_t>(slave_no_, 0x3033, 0x0, 0x33000000);
                usleep(30000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp.message="Channel 2 phase 1 failed";
                resp.success=false;
                writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2033, 0x0)==0
               && manager_->readSDO<int32_t>(slave_no_, 0x2034, 0x0)==0)
            {
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp.message="Channel 2 phase 2 failed";
                resp.success=false;
                writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
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
        writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
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

    //channel1
    if((readInput_half_unit(elfin_txpdo::AXIS1_STATUSWORD_L16, false) & 0xc) == 0)
    {
        manager_->writeSDO<int8_t>(slave_no_, 0x6060, 0x0, 0xb);
        writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0xb, true, false);
        usleep(20000);
        manager_->writeSDO<int32_t>(slave_no_, 0x3023, 0x0, 0x22000000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2023, 0x0)==0x400000
               && manager_->readSDO<int32_t>(slave_no_, 0x2024, 0x0)==0x400000)
            {
                usleep(20000);
                manager_->writeSDO<int32_t>(slave_no_, 0x3023, 0x0, 0x33000000);
                usleep(30000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp.message="Channel 1 phase 1 failed";
                resp.success=false;
                writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2023, 0x0)==0
               && manager_->readSDO<int32_t>(slave_no_, 0x2024, 0x0)==0)
            {
                manager_->writeSDO<int32_t>(slave_no_, 0x3023, 0x0, 0x0);
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp.message="Channel 1 phase 2 failed";
                resp.success=false;
                writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        manager_->writeSDO<int8_t>(slave_no_, 0x6060, 0x0, 0x8);
        writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        usleep(50000);
    }
    else
    {
        resp.message="Channel 1 failed, the reason might be there is a fault or the motor is enabled";
        resp.success=false;
        writeOutput_unit_byte(elfin_rxpdo::AXIS1_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        return true;
    }

    //channel2
    if((readInput_half_unit(elfin_txpdo::AXIS2_STATUSWORD_L16, false) & 0xc) == 0)
    {
        manager_->writeSDO<int8_t>(slave_no_, 0x6860, 0x0, 0xb);
        writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0xb, true, false);
        usleep(20000);
        manager_->writeSDO<int32_t>(slave_no_, 0x3033, 0x0, 0x22000000);
        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2033, 0x0)==0x400000
               && manager_->readSDO<int32_t>(slave_no_, 0x2034, 0x0)==0x400000)
            {
                usleep(20000);
                manager_->writeSDO<int32_t>(slave_no_, 0x3033, 0x0, 0x33000000);
                usleep(30000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp.message="Channel 2 phase 1 failed";
                resp.success=false;
                writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while(ros::ok())
        {
            if(manager_->readSDO<int32_t>(slave_no_, 0x2033, 0x0)==0
               && manager_->readSDO<int32_t>(slave_no_, 0x2034, 0x0)==0)
            {
                manager_->writeSDO<int32_t>(slave_no_, 0x3033, 0x0, 0x0);
                usleep(50000);
                break;
            }
            if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
            {
                resp.message="Channel 2 phase 2 failed";
                resp.success=false;
                writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
                return true;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        manager_->writeSDO<int8_t>(slave_no_, 0x6860, 0x0, 0x8);
        writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        usleep(50000);
    }
    else
    {
        resp.message="Channel 2 failed, the reason might be there is a fault or the motor is enabled";
        resp.success=false;
        writeOutput_unit_byte(elfin_rxpdo::AXIS2_MODES_OF_OPERATION_BYTE2, 0x8, true, false);
        return true;
    }

    resp.message="band-type brake is closed";
    resp.success=true;
    return true;
}

}

