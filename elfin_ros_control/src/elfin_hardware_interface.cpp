/*
Created on Wed Oct 25 11:23:42 2017

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
        module_infos_[i].axis1.count_Nm_factor=module_infos_[i].axis1.axis_torque_factor/module_infos_[i].axis1.reduction_ratio;

        module_infos_[i].axis2.count_rad_factor=module_infos_[i].axis2.reduction_ratio*module_infos_[i].axis2.axis_position_factor/(2*M_PI);
        module_infos_[i].axis2.count_Nm_factor=module_infos_[i].axis2.axis_torque_factor/module_infos_[i].axis2.reduction_ratio;
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
}

ElfinHWInterface::~ElfinHWInterface()
{
    for(int i=0; i<ethercat_drivers_.size(); i++)
    {
        if(ethercat_drivers_[i]!=NULL)
            delete ethercat_drivers_[i];
    }
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
                for(int j=0; j<module_infos_.size(); j++)
                {
                    if(stop_resrcs[i].resources.find(module_infos_[j].axis1.name)!=stop_resrcs[i].resources.end()
                       || stop_resrcs[i].resources.find(module_infos_[j].axis2.name)!=stop_resrcs[i].resources.end())
                    {
                        if(module_infos_[j].client_ptr->isEnabled())
                        {
                            module_infos_[j].client_ptr->setPosMode();
                            if(!(module_infos_[j].client_ptr->isEnabled() && module_infos_[j].client_ptr->inPosMode()))
                            {
                                ROS_ERROR("can't stop %s, module[%i]: set position mode failed", iter->name.c_str(), j);
                                return false;
                            }
                        }
                    }
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
            for(int j=0; j<module_infos_.size(); j++)
            {
                if(start_resrcs[i].resources.find(module_infos_[j].axis1.name)!=start_resrcs[i].resources.end()
                   || start_resrcs[i].resources.find(module_infos_[j].axis2.name)!=start_resrcs[i].resources.end())
                {
                    if(!module_infos_[j].client_ptr->isEnabled())
                    {
                        ROS_ERROR("can't start %s, because module[%i] is not enabled", iter->name.c_str(), j);
                        return false;
                    }

                    if(strcmp(start_resrcs[i].hardware_interface.c_str(), "hardware_interface::PositionJointInterface")==0)
                    {
                        module_infos_[j].client_ptr->setPosMode();
                        if(!(module_infos_[j].client_ptr->isEnabled() && module_infos_[j].client_ptr->inPosMode()))
                        {
                            ROS_ERROR("module[%i]: set position mode failed", j);
                            return false;
                        }
                    }

                    else if(strcmp(start_resrcs[i].hardware_interface.c_str(), "hardware_interface::EffortJointInterface")==0)
                    {
                        module_infos_[j].client_ptr->setTrqMode();
                        if(!(module_infos_[j].client_ptr->isEnabled() && module_infos_[j].client_ptr->inTrqMode()))
                        {
                            ROS_ERROR("module[%i]: set torque mode failed", j);
                            return false;
                        }
                    }

                    else
                    {
                        ROS_ERROR("module[%i] doesn't support %s", j, start_resrcs[i].hardware_interface.c_str());
                        return false;
                    }
                }
            }
        }
    }

    return true;
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
        int16_t trq_count1=module_infos_[i].client_ptr->getAxis1TrqCnt();
        int32_t pos_count_diff_1=pos_count1-module_infos_[i].axis1.count_zero;

        double position_tmp1=-1*pos_count_diff_1/module_infos_[i].axis1.count_rad_factor;
        module_infos_[i].axis1.velocity=(position_tmp1-module_infos_[i].axis1.position)/read_update_dur_.toSec();
        module_infos_[i].axis1.position=position_tmp1;
        module_infos_[i].axis1.effort=-1*trq_count1/module_infos_[i].axis1.count_Nm_factor;

        int32_t pos_count2=module_infos_[i].client_ptr->getAxis2PosCnt();
        int16_t trq_count2=module_infos_[i].client_ptr->getAxis2TrqCnt();
        int32_t pos_count_diff_2=pos_count2-module_infos_[i].axis2.count_zero;

        double position_tmp2=-1*pos_count_diff_2/module_infos_[i].axis2.count_rad_factor;
        module_infos_[i].axis2.velocity=(position_tmp2-module_infos_[i].axis2.position)/read_update_dur_.toSec();
        module_infos_[i].axis2.position=position_tmp2;
        module_infos_[i].axis2.effort=-1*trq_count2/module_infos_[i].axis2.count_Nm_factor;
    }

}

void ElfinHWInterface::write_update()
{
    for(size_t i=0; i<module_infos_.size(); i++)
    {
        if(!(module_infos_[i].client_ptr->isEnabled() && module_infos_[i].client_ptr->inPosBasedMode()))
        {
            module_infos_[i].axis1.position_cmd=module_infos_[i].axis1.position;
            module_infos_[i].axis2.position_cmd=module_infos_[i].axis2.position;
        }
        double position_cmd_count1=-1 * module_infos_[i].axis1.position_cmd * module_infos_[i].axis1.count_rad_factor + module_infos_[i].axis1.count_zero;
        double position_cmd_count2=-1 * module_infos_[i].axis2.position_cmd * module_infos_[i].axis2.count_rad_factor + module_infos_[i].axis2.count_zero;

        module_infos_[i].client_ptr->setAxis1PosCnt(int32_t(position_cmd_count1));
        module_infos_[i].client_ptr->setAxis2PosCnt(int32_t(position_cmd_count2));

        double torque_cmd_count1=-1 * module_infos_[i].axis1.effort_cmd * module_infos_[i].axis1.count_Nm_factor;
        double torque_cmd_count2=-1 * module_infos_[i].axis2.effort_cmd * module_infos_[i].axis2.count_Nm_factor;

        module_infos_[i].client_ptr->setAxis1TrqCnt(int16_t(torque_cmd_count1));
        module_infos_[i].client_ptr->setAxis2TrqCnt(int16_t(torque_cmd_count2));
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
