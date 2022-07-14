/*
Created on Tue Sep 25 10:16 2018

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
// update the include file
#include "elfin_ros_control/elfin_hardware_interface.h"

namespace elfin_ros_control {

ElfinHWInterface::ElfinHWInterface(elfin_ethercat_driver::EtherCatManager *manager, const ros::NodeHandle &nh):
    n_(nh)
{
    //Initialize elfin_driver_names_
    std::vector<std::string> elfin_driver_names_default;
    elfin_driver_names_default.resize(1);
    elfin_driver_names_default[0]="elfin";
    n_.param<std::vector<std::string> >("elfin_ethercat_drivers", elfin_driver_names_, elfin_driver_names_default);

    // Initialize ethercat_drivers_
    ethercat_drivers_.clear();
    ethercat_drivers_.resize(elfin_driver_names_.size());
    for(int i=0; i<ethercat_drivers_.size(); i++)
    {
        ethercat_drivers_[i]=new elfin_ethercat_driver::ElfinEtherCATDriver(manager, elfin_driver_names_[i]);
    }

    // Initialize module_infos_
    module_infos_.clear();
    for(size_t i=0; i<ethercat_drivers_.size(); i++)
    {
        for(size_t j=0; j<ethercat_drivers_[i]->getEtherCATClientNumber(); j++)
        {
            ModuleInfo module_info_tmp;
            module_info_tmp.client_ptr=ethercat_drivers_[i]->getEtherCATClientPtr(j);

            module_info_tmp.axis1.name=ethercat_drivers_[i]->getJointName(2*j);
            module_info_tmp.axis1.reduction_ratio=ethercat_drivers_[i]->getReductionRatio(2*j);
            module_info_tmp.axis1.axis_position_factor=ethercat_drivers_[i]->getAxisPositionFactor(2*j);
            module_info_tmp.axis1.count_zero=ethercat_drivers_[i]->getCountZero(2*j);
            module_info_tmp.axis1.axis_torque_factor=ethercat_drivers_[i]->getAxisTorqueFactor(2*j);

            module_info_tmp.axis2.name=ethercat_drivers_[i]->getJointName(2*j+1);
            module_info_tmp.axis2.reduction_ratio=ethercat_drivers_[i]->getReductionRatio(2*j+1);
            module_info_tmp.axis2.axis_position_factor=ethercat_drivers_[i]->getAxisPositionFactor(2*j+1);
            module_info_tmp.axis2.count_zero=ethercat_drivers_[i]->getCountZero(2*j+1);
            module_info_tmp.axis2.axis_torque_factor=ethercat_drivers_[i]->getAxisTorqueFactor(2*j+1);

            module_infos_.push_back(module_info_tmp);
        }
    }

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        module_infos_[i].axis1.count_rad_factor=module_infos_[i].axis1.reduction_ratio*module_infos_[i].axis1.axis_position_factor/(2*M_PI);
        module_infos_[i].axis1.count_rad_per_s_factor=module_infos_[i].axis1.count_rad_factor/750.3;
        module_infos_[i].axis1.count_Nm_factor=module_infos_[i].axis1.axis_torque_factor/module_infos_[i].axis1.reduction_ratio;

        module_infos_[i].axis2.count_rad_factor=module_infos_[i].axis2.reduction_ratio*module_infos_[i].axis2.axis_position_factor/(2*M_PI);
        module_infos_[i].axis2.count_rad_per_s_factor=module_infos_[i].axis2.count_rad_factor/750.3;
        module_infos_[i].axis2.count_Nm_factor=module_infos_[i].axis2.axis_torque_factor/module_infos_[i].axis2.reduction_ratio;
    }

    // Initialize pre_switch_flags_ and pre_switch_mutex_ptrs_
    pre_switch_flags_.resize(module_infos_.size());
    for(int i=0; i<pre_switch_flags_.size(); i++)
    {
        pre_switch_flags_[i]=false;
    }

    pre_switch_mutex_ptrs_.resize(module_infos_.size());
    for(int i=0; i<pre_switch_mutex_ptrs_.size(); i++)
    {
        pre_switch_mutex_ptrs_[i]=boost::shared_ptr<boost::mutex>(new boost::mutex);
    }

    // Initialize the state and command interface
    for(size_t i=0; i<module_infos_.size(); i++)
    {
        hardware_interface::JointStateHandle jnt_state_handle_tmp1(module_infos_[i].axis1.name,
                                                                  &module_infos_[i].axis1.position,
                                                                  &module_infos_[i].axis1.velocity,
                                                                  &module_infos_[i].axis1.effort);
        jnt_state_interface_.registerHandle(jnt_state_handle_tmp1);

        hardware_interface::JointStateHandle jnt_state_handle_tmp2(module_infos_[i].axis2.name,
                                                                  &module_infos_[i].axis2.position,
                                                                  &module_infos_[i].axis2.velocity,
                                                                  &module_infos_[i].axis2.effort);
        jnt_state_interface_.registerHandle(jnt_state_handle_tmp2);
    }
    registerInterface(&jnt_state_interface_);

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        hardware_interface::JointHandle jnt_handle_tmp1(jnt_state_interface_.getHandle(module_infos_[i].axis1.name),
                                                       &module_infos_[i].axis1.position_cmd);
        jnt_position_cmd_interface_.registerHandle(jnt_handle_tmp1);

        hardware_interface::JointHandle jnt_handle_tmp2(jnt_state_interface_.getHandle(module_infos_[i].axis2.name),
                                                       &module_infos_[i].axis2.position_cmd);
        jnt_position_cmd_interface_.registerHandle(jnt_handle_tmp2);
    }
    registerInterface(&jnt_position_cmd_interface_);

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        hardware_interface::JointHandle jnt_handle_tmp1(jnt_state_interface_.getHandle(module_infos_[i].axis1.name),
                                                        &module_infos_[i].axis1.effort_cmd);
        jnt_effort_cmd_interface_.registerHandle(jnt_handle_tmp1);

        hardware_interface::JointHandle jnt_handle_tmp2(jnt_state_interface_.getHandle(module_infos_[i].axis2.name),
                                                        &module_infos_[i].axis2.effort_cmd);
        jnt_effort_cmd_interface_.registerHandle(jnt_handle_tmp2);

    }
    registerInterface(&jnt_effort_cmd_interface_);

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        elfin_hardware_interface::PosTrqJointHandle jnt_handle_tmp1(jnt_state_interface_.getHandle(module_infos_[i].axis1.name),
                                                                    &module_infos_[i].axis1.position_cmd,
                                                                    &module_infos_[i].axis1.effort_cmd);
        jnt_postrq_cmd_interface_.registerHandle(jnt_handle_tmp1);

        elfin_hardware_interface::PosTrqJointHandle jnt_handle_tmp2(jnt_state_interface_.getHandle(module_infos_[i].axis2.name),
                                                                    &module_infos_[i].axis2.position_cmd,
                                                                    &module_infos_[i].axis2.effort_cmd);
        jnt_postrq_cmd_interface_.registerHandle(jnt_handle_tmp2);
    }
    registerInterface(&jnt_postrq_cmd_interface_);

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        hardware_interface::PosVelJointHandle jnt_handle_tmp1(jnt_state_interface_.getHandle(module_infos_[i].axis1.name),
                                                                    &module_infos_[i].axis1.position_cmd,
                                                                    &module_infos_[i].axis1.vel_ff_cmd);
        jnt_posvel_cmd_interface_.registerHandle(jnt_handle_tmp1);

        hardware_interface::PosVelJointHandle jnt_handle_tmp2(jnt_state_interface_.getHandle(module_infos_[i].axis2.name),
                                                                    &module_infos_[i].axis2.position_cmd,
                                                                    &module_infos_[i].axis2.vel_ff_cmd);
        jnt_posvel_cmd_interface_.registerHandle(jnt_handle_tmp2);
    }
    registerInterface(&jnt_posvel_cmd_interface_);

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        elfin_hardware_interface::PosVelTrqJointHandle jnt_handle_tmp1(jnt_state_interface_.getHandle(module_infos_[i].axis1.name),
                                                                       &module_infos_[i].axis1.position_cmd,
                                                                       &module_infos_[i].axis1.vel_ff_cmd,
                                                                       &module_infos_[i].axis1.effort_cmd);
        jnt_posveltrq_cmd_interface_.registerHandle(jnt_handle_tmp1);

        elfin_hardware_interface::PosVelTrqJointHandle jnt_handle_tmp2(jnt_state_interface_.getHandle(module_infos_[i].axis2.name),
                                                                       &module_infos_[i].axis2.position_cmd,
                                                                       &module_infos_[i].axis2.vel_ff_cmd,
                                                                       &module_infos_[i].axis2.effort_cmd);
        jnt_posveltrq_cmd_interface_.registerHandle(jnt_handle_tmp2);
    }
    registerInterface(&jnt_posveltrq_cmd_interface_);

    // Initialize motion_threshold_
    motion_threshold_=5e-5;
}

ElfinHWInterface::~ElfinHWInterface()
{
    for(int i=0; i<ethercat_drivers_.size(); i++)
    {
        if(ethercat_drivers_[i]!=NULL)
            delete ethercat_drivers_[i];
    }
}

bool ElfinHWInterface::isModuleMoving(int module_num)
{
    std::vector<double> previous_pos;
    std::vector<double> last_pos;

    previous_pos.resize(2);
    last_pos.resize(2);

    int32_t count1, count2;

    module_infos_[module_num].client_ptr->getActPosCounts(count1, count2);
    previous_pos[0]=count1/module_infos_[module_num].axis1.count_rad_factor;
    previous_pos[1]=count2/module_infos_[module_num].axis2.count_rad_factor;

    usleep(10000);

    module_infos_[module_num].client_ptr->getActPosCounts(count1, count2);
    last_pos[0]=count1/module_infos_[module_num].axis1.count_rad_factor;
    last_pos[1]=count2/module_infos_[module_num].axis2.count_rad_factor;

    for(int i=0; i<previous_pos.size(); i++)
    {
        if(fabs(last_pos[i]-previous_pos[i])>motion_threshold_)
        {
            return true;
        }
    }

    return false;
}

bool ElfinHWInterface::setGroupPosMode(const std::vector<int> &module_no)
{
    for(int j=0; j<module_no.size(); j++)
    {
        boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[j]]);
        pre_switch_flags_[module_no[j]]=true;
        pre_switch_flags_lock.unlock();
    }

    usleep(5000);

    for(int j=0; j<module_no.size(); j++)
    {
        module_infos_[module_no[j]].client_ptr->setAxis1VelFFCnt(0x0);
        module_infos_[module_no[j]].client_ptr->setAxis2VelFFCnt(0x0);

        module_infos_[module_no[j]].client_ptr->setAxis1TrqCnt(0x0);
        module_infos_[module_no[j]].client_ptr->setAxis2TrqCnt(0x0);
    }

    for(int j=0; j<module_no.size(); j++)
    {
        module_infos_[module_no[j]].client_ptr->setPosMode();
    }

    usleep(10000);

    for(int j=0; j<module_no.size(); j++)
    {
        if(!module_infos_[module_no[j]].client_ptr->inPosMode())
        {
            ROS_ERROR("module[%i]: set position mode failed", module_no[j]);

            for(int k=0; k<module_no.size(); k++)
            {
                boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[k]]);
                pre_switch_flags_[module_no[k]]=false;
                pre_switch_flags_lock.unlock();
            }

            return false;
        }
    }

    return true;
}

bool ElfinHWInterface::setGroupTrqMode(const std::vector<int> &module_no)
{
    for(int j=0; j<module_no.size(); j++)
    {
        boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[j]]);
        pre_switch_flags_[module_no[j]]=true;
        pre_switch_flags_lock.unlock();
    }

    usleep(5000);

    for(int j=0; j<module_no.size(); j++)
    {
        module_infos_[module_no[j]].client_ptr->setAxis1TrqCnt(0x0);
        module_infos_[module_no[j]].client_ptr->setAxis2TrqCnt(0x0);
    }

    for(int j=0; j<module_no.size(); j++)
    {
        module_infos_[module_no[j]].client_ptr->setTrqMode();
    }

    usleep(10000);

    bool set_trq_success=true;
    for(int j=0; j<module_no.size(); j++)
    {
        if(!module_infos_[module_no[j]].client_ptr->inTrqMode())
        {
            ROS_ERROR("module[%i]: set torque mode failed, setting position mode", module_no[j]);
            set_trq_success=false;
            break;
        }
    }

    if(!set_trq_success)
    {
        for(int j=0; j<module_no.size(); j++)
        {
            module_infos_[module_no[j]].client_ptr->setPosMode();
        }

        usleep(10000);

        for(int j=0; j<module_no.size(); j++)
        {
            if(!module_infos_[module_no[j]].client_ptr->inPosMode())
            {
                ROS_ERROR("module[%i]: set position mode failed too", module_no[j]);
            }
        }

        for(int j=0; j<module_no.size(); j++)
        {
            boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[j]]);
            pre_switch_flags_[module_no[j]]=false;
            pre_switch_flags_lock.unlock();
        }

        return false;
    }

    return true;
}

bool ElfinHWInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                     const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    std::list<hardware_interface::ControllerInfo>::const_iterator iter;

    if(!stop_list.empty())
    {
        for(iter=stop_list.begin(); iter!=stop_list.end(); iter++)
        {
            std::vector<hardware_interface::InterfaceResources> stop_resrcs=iter->claimed_resources;
            for(int i=0; i<stop_resrcs.size(); i++)
            {
                std::vector<int> module_no;
                module_no.clear();

                for(int j=0; j<module_infos_.size(); j++)
                {
                    if(stop_resrcs[i].resources.find(module_infos_[j].axis1.name)!=stop_resrcs[i].resources.end()
                       || stop_resrcs[i].resources.find(module_infos_[j].axis2.name)!=stop_resrcs[i].resources.end())
                    {
                        if(module_infos_[j].client_ptr->isEnabled())
                        {
                            module_no.push_back(j);
                        }
                    }
                }

                std::vector<int> module_no_tmp=module_no;

                if(!setGroupPosMode(module_no_tmp))
                {
                    return false;
                }
            }
        }
    }

    if(start_list.empty())
    return true;

    for(iter=start_list.begin(); iter!=start_list.end(); iter++)
    {
        std::vector<hardware_interface::InterfaceResources> start_resrcs=iter->claimed_resources;
        for(int i=0; i<start_resrcs.size(); i++)
        {
            std::vector<int> module_no;
            module_no.clear();

            for(int j=0; j<module_infos_.size(); j++)
            {
                bool axis1_exist=(start_resrcs[i].resources.find(module_infos_[j].axis1.name)!=start_resrcs[i].resources.end());
                bool axis2_exist=(start_resrcs[i].resources.find(module_infos_[j].axis2.name)!=start_resrcs[i].resources.end());

                if(axis1_exist || axis2_exist)
                {
                    if(axis1_exist && axis2_exist)
                    {
                        if(!module_infos_[j].client_ptr->isEnabled())
                        {
                            ROS_ERROR("can't start %s, because module[%i] is not enabled", iter->name.c_str(), j);
                            return false;
                        }

                        if(isModuleMoving(j))
                        {
                            ROS_ERROR("can't start %s, because module[%i] is moving", iter->name.c_str(), j);
                            return false;
                        }

                        module_no.push_back(j);
                    }
                    else if(axis1_exist)
                    {
                        ROS_ERROR("If %s includes %s, it should include %s too", iter->name.c_str(),
                                  module_infos_[j].axis1.name.c_str(), module_infos_[j].axis2.name.c_str());
                        return false;
                    }
                    else
                    {
                        ROS_ERROR("If %s includes %s, it should include %s too", iter->name.c_str(),
                                  module_infos_[j].axis2.name.c_str(), module_infos_[j].axis1.name.c_str());
                        return false;
                    }
                }
            }

            if(strcmp(start_resrcs[i].hardware_interface.c_str(), "hardware_interface::PositionJointInterface")==0)
            {
                std::vector<int> module_no_tmp=module_no;

                if(!setGroupPosMode(module_no_tmp))
                {
                    return false;
                }
            }

            else if(strcmp(start_resrcs[i].hardware_interface.c_str(), "hardware_interface::PosVelJointInterface")==0)
            {
                std::vector<int> module_no_tmp=module_no;

                if(!setGroupPosMode(module_no_tmp))
                {
                    return false;
                }
            }

            else if(strcmp(start_resrcs[i].hardware_interface.c_str(), "elfin_hardware_interface::PosTrqJointInterface")==0)
            {
                std::vector<int> module_no_tmp=module_no;

                if(!setGroupPosMode(module_no_tmp))
                {
                    return false;
                }
            }

            else if(strcmp(start_resrcs[i].hardware_interface.c_str(), "elfin_hardware_interface::PosVelTrqJointInterface")==0)
            {
                std::vector<int> module_no_tmp=module_no;

                if(!setGroupPosMode(module_no_tmp))
                {
                    return false;
                }
            }

            else if(strcmp(start_resrcs[i].hardware_interface.c_str(), "hardware_interface::EffortJointInterface")==0)
            {
                std::vector<int> module_no_tmp=module_no;
                if(!setGroupTrqMode(module_no_tmp))
                {
                    return false;
                }
            }

            else
            {
                if(!module_no.empty())
                {
                  ROS_ERROR("Elfin doesn't support %s", start_resrcs[i].hardware_interface.c_str());
                  return false;
                }
            }
        }
    }

    return true;
}

void ElfinHWInterface::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    for(size_t i=0; i<pre_switch_flags_.size(); i++)
    {
        boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[i]);
        if(pre_switch_flags_[i])
        {
            module_infos_[i].axis1.velocity_cmd=0;
            module_infos_[i].axis1.vel_ff_cmd=0;
            module_infos_[i].axis1.effort_cmd=0;

            module_infos_[i].axis2.velocity_cmd=0;
            module_infos_[i].axis2.vel_ff_cmd=0;
            module_infos_[i].axis2.effort_cmd=0;

            pre_switch_flags_[i]=false;
        }
        pre_switch_flags_lock.unlock();
    }
}

void ElfinHWInterface::read_init()
{
    struct timespec read_update_tick;
    clock_gettime(CLOCK_REALTIME, &read_update_tick);
    read_update_time_.sec=read_update_tick.tv_sec;
    read_update_time_.nsec=read_update_tick.tv_nsec;

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        int32_t pos_count1=module_infos_[i].client_ptr->getAxis1PosCnt();
        double position_tmp_1=(pos_count1-module_infos_[i].axis1.count_zero)/module_infos_[i].axis1.count_rad_factor;
        if(position_tmp_1>=M_PI)
        {
            module_infos_[i].axis1.count_zero+=module_infos_[i].axis1.count_rad_factor*2*M_PI;
        }
        else if(position_tmp_1<-1*M_PI)
        {
            module_infos_[i].axis1.count_zero-=module_infos_[i].axis1.count_rad_factor*2*M_PI;
        }
        module_infos_[i].axis1.position=-1*(pos_count1-module_infos_[i].axis1.count_zero)/module_infos_[i].axis1.count_rad_factor;

        int32_t pos_count2=module_infos_[i].client_ptr->getAxis2PosCnt();
        double position_tmp_2=(pos_count2-module_infos_[i].axis2.count_zero)/module_infos_[i].axis2.count_rad_factor;
        if(position_tmp_2>=M_PI)
        {
            module_infos_[i].axis2.count_zero+=module_infos_[i].axis2.count_rad_factor*2*M_PI;
        }
        else if(position_tmp_2<-1*M_PI)
        {
            module_infos_[i].axis2.count_zero-=module_infos_[i].axis2.count_rad_factor*2*M_PI;
        }
        module_infos_[i].axis2.position=-1*(pos_count2-module_infos_[i].axis2.count_zero)/module_infos_[i].axis2.count_rad_factor;
    }

}

void ElfinHWInterface::read_update(const ros::Time &time_now)
{
    read_update_dur_=time_now - read_update_time_;
    read_update_time_=time_now;

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        int32_t pos_count1=module_infos_[i].client_ptr->getAxis1PosCnt();
        int16_t vel_count1=module_infos_[i].client_ptr->getAxis1VelCnt();
        int16_t trq_count1=module_infos_[i].client_ptr->getAxis1TrqCnt();
        int32_t pos_count_diff_1=pos_count1-module_infos_[i].axis1.count_zero;

        double position_tmp1=-1*pos_count_diff_1/module_infos_[i].axis1.count_rad_factor;
        module_infos_[i].axis1.position=position_tmp1;
        module_infos_[i].axis1.velocity=-1*vel_count1/module_infos_[i].axis1.count_rad_per_s_factor;
        module_infos_[i].axis1.effort=-1*trq_count1/module_infos_[i].axis1.count_Nm_factor;

        int32_t pos_count2=module_infos_[i].client_ptr->getAxis2PosCnt();
        int16_t vel_count2=module_infos_[i].client_ptr->getAxis2VelCnt();
        int16_t trq_count2=module_infos_[i].client_ptr->getAxis2TrqCnt();
        int32_t pos_count_diff_2=pos_count2-module_infos_[i].axis2.count_zero;

        double position_tmp2=-1*pos_count_diff_2/module_infos_[i].axis2.count_rad_factor;
        module_infos_[i].axis2.position=position_tmp2;
        module_infos_[i].axis2.velocity=-1*vel_count2/module_infos_[i].axis2.count_rad_per_s_factor;
        module_infos_[i].axis2.effort=-1*trq_count2/module_infos_[i].axis2.count_Nm_factor;
    }

}

void ElfinHWInterface::write_update()
{
    for(size_t i=0; i<module_infos_.size(); i++)
    {
        if(!module_infos_[i].client_ptr->inPosBasedMode())
        {
            module_infos_[i].axis1.position_cmd=module_infos_[i].axis1.position;
            module_infos_[i].axis2.position_cmd=module_infos_[i].axis2.position;
        }
        double position_cmd_count1=-1 * module_infos_[i].axis1.position_cmd * module_infos_[i].axis1.count_rad_factor + module_infos_[i].axis1.count_zero;
        double position_cmd_count2=-1 * module_infos_[i].axis2.position_cmd * module_infos_[i].axis2.count_rad_factor + module_infos_[i].axis2.count_zero;

        module_infos_[i].client_ptr->setAxis1PosCnt(int32_t(position_cmd_count1));
        module_infos_[i].client_ptr->setAxis2PosCnt(int32_t(position_cmd_count2));

        bool is_preparing_switch;
        boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[i]);
        is_preparing_switch=pre_switch_flags_[i];
        pre_switch_flags_lock.unlock();

        if(!is_preparing_switch)
        {
            double vel_ff_cmd_count1=-1 * module_infos_[i].axis1.vel_ff_cmd * module_infos_[i].axis1.count_rad_per_s_factor / 16.25;
            double vel_ff_cmd_count2=-1 * module_infos_[i].axis2.vel_ff_cmd * module_infos_[i].axis2.count_rad_per_s_factor / 16.25;

            module_infos_[i].client_ptr->setAxis1VelFFCnt(int16_t(vel_ff_cmd_count1));
            module_infos_[i].client_ptr->setAxis2VelFFCnt(int16_t(vel_ff_cmd_count2));

            double torque_cmd_count1=-1 * module_infos_[i].axis1.effort * module_infos_[i].axis1.count_Nm_factor;
            double torque_cmd_count2=-1 * module_infos_[i].axis2.effort * module_infos_[i].axis2.count_Nm_factor;

            module_infos_[i].client_ptr->setAxis1TrqCnt(int16_t(torque_cmd_count1));
            module_infos_[i].client_ptr->setAxis2TrqCnt(int16_t(torque_cmd_count2));
        }
    }
}

} // end namespace elfin_ros_control

typedef struct{
    controller_manager::ControllerManager *manager;
    elfin_ros_control::ElfinHWInterface *elfin_hw_interface;
}ArgsForThread;

static void timespecInc(struct timespec &tick, int nsec)
{
  int SEC_2_NSEC = 1e+9;
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= SEC_2_NSEC)
  {
    tick.tv_nsec -= SEC_2_NSEC;
    ++tick.tv_sec;
  }
}

void* update_loop(void* threadarg)
{
    ArgsForThread *arg=(ArgsForThread *)threadarg;
    controller_manager::ControllerManager *manager=arg->manager;
    elfin_ros_control::ElfinHWInterface *interface=arg->elfin_hw_interface;
    ros::Duration d(0.001);
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);
    //time for checking overrun
    struct timespec before;
    double overrun_time;
    while(ros::ok())
    {
        ros::Time this_moment(tick.tv_sec, tick.tv_nsec);
        interface->read_update(this_moment);
        manager->update(this_moment, d);
        interface->write_update();
        timespecInc(tick, d.nsec);
        // check overrun
        clock_gettime(CLOCK_REALTIME, &before);
        overrun_time = (before.tv_sec + double(before.tv_nsec)/1e+9) -  (tick.tv_sec + double(tick.tv_nsec)/1e+9);
        if(overrun_time > 0.0)
        {
            tick.tv_sec=before.tv_sec;
            tick.tv_nsec=before.tv_nsec;
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"elfin_hardware_interface", ros::init_options::AnonymousName);

    ros::NodeHandle nh("~");
    std::string ethernet_name;
    ethernet_name=nh.param<std::string>("elfin_ethernet_name", "eth0");

    elfin_ethercat_driver::EtherCatManager em(ethernet_name);
    elfin_ros_control::ElfinHWInterface elfin_hw(&em);
    elfin_hw.read_init();

    controller_manager::ControllerManager cm(&elfin_hw);
    pthread_t tid;
    ArgsForThread *thread_arg=new ArgsForThread();
    thread_arg->manager=&cm;
    thread_arg->elfin_hw_interface=&elfin_hw;
    pthread_create(&tid, NULL, update_loop, thread_arg);

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
