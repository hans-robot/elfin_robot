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
    n_.param<std::vector<std::string> >("elfin_ethercat_drivers", elfin_driver_names, elfin_driver_names_default);

    // Initialize ethercat_drivers_
    ethercat_drivers_.clear();
    ethercat_drivers_.resize(elfin_driver_names.size());
    for(int i=0; i<ethercat_drivers_.size(); i++)
    {
        ethercat_drivers_[i]=new elfin_ethercat_driver::ElfinEtherCATDriver(manager, elfin_driver_names[i]);
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
            module_info_tmp.axis1.has_torque_mode=ethercat_drivers_[i]->hasTorqueMode();
            if(module_info_tmp.axis1.has_torque_mode)
                module_info_tmp.axis1.axis_torque_factor=ethercat_drivers_[i]->getAxisTorqueFactor(2*j);

            module_info_tmp.axis2.name=ethercat_drivers_[i]->getJointName(2*j+1);
            module_info_tmp.axis2.reduction_ratio=ethercat_drivers_[i]->getReductionRatio(2*j+1);
            module_info_tmp.axis2.axis_position_factor=ethercat_drivers_[i]->getAxisPositionFactor(2*j+1);
            module_info_tmp.axis2.count_zero=ethercat_drivers_[i]->getCountZero(2*j+1);
            module_info_tmp.axis2.has_torque_mode=ethercat_drivers_[i]->hasTorqueMode();
            if(module_info_tmp.axis2.has_torque_mode)
                module_info_tmp.axis2.axis_torque_factor=ethercat_drivers_[i]->getAxisTorqueFactor(2*j+1);

            module_infos_.push_back(module_info_tmp);
        }
    }

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        module_infos_[i].axis1.count_rad_factor=module_infos_[i].axis1.reduction_ratio*module_infos_[i].axis1.axis_position_factor/(2*M_PI);
        if(module_infos_[i].axis1.has_torque_mode)
            module_infos_[i].axis1.count_Nm_factor=module_infos_[i].axis1.axis_torque_factor/module_infos_[i].axis1.reduction_ratio;

        module_infos_[i].axis2.count_rad_factor=module_infos_[i].axis2.reduction_ratio*module_infos_[i].axis2.axis_position_factor/(2*M_PI);
        if(module_infos_[i].axis2.has_torque_mode)
            module_infos_[i].axis2.count_Nm_factor=module_infos_[i].axis2.axis_torque_factor/module_infos_[i].axis2.reduction_ratio;
    }

    // Initialize the state and command interface
    for(size_t i=0; i<module_infos_.size(); i++)
    {
        hardware_interface::JointStateHandle jnt_state_handle_tmp1(module_infos_[i].axis1.name,
                                                                  &module_infos_[i].axis1.position,
                                                                  &module_infos_[i].axis1.velocity,
                                                                  &module_infos_[i].axis1.effort);
        jnt_state_interface.registerHandle(jnt_state_handle_tmp1);

        hardware_interface::JointStateHandle jnt_state_handle_tmp2(module_infos_[i].axis2.name,
                                                                  &module_infos_[i].axis2.position,
                                                                  &module_infos_[i].axis2.velocity,
                                                                  &module_infos_[i].axis2.effort);
        jnt_state_interface.registerHandle(jnt_state_handle_tmp2);
    }
    registerInterface(&jnt_state_interface);

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        hardware_interface::JointHandle jnt_handle_tmp1(jnt_state_interface.getHandle(module_infos_[i].axis1.name),
                                                       &module_infos_[i].axis1.position_cmd);
        jnt_cmd_interface.registerHandle(jnt_handle_tmp1);

        hardware_interface::JointHandle jnt_handle_tmp2(jnt_state_interface.getHandle(module_infos_[i].axis2.name),
                                                       &module_infos_[i].axis2.position_cmd);
        jnt_cmd_interface.registerHandle(jnt_handle_tmp2);
    }
    registerInterface(&jnt_cmd_interface);
}

ElfinHWInterface::~ElfinHWInterface()
{
    for(int i=0; i<ethercat_drivers_.size(); i++)
    {
        if(ethercat_drivers_[i]!=NULL)
            delete ethercat_drivers_[i];
    }
}

void ElfinHWInterface::read_init()
{
    struct timespec read_update_tick;
    clock_gettime(CLOCK_REALTIME, &read_update_tick);
    read_update_time.sec=read_update_tick.tv_sec;
    read_update_time.nsec=read_update_tick.tv_nsec;

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
    read_update_dur=time_now - read_update_time;
    read_update_time=time_now;

    for(size_t i=0; i<module_infos_.size(); i++)
    {
        int32_t pos_count1=module_infos_[i].client_ptr->getAxis1PosCnt();
        int16_t trq_count1=module_infos_[i].client_ptr->getAxis1TrqCnt();
        int32_t pos_count_diff_1=pos_count1-module_infos_[i].axis1.count_zero;

        double position_tmp1=-1*pos_count_diff_1/module_infos_[i].axis1.count_rad_factor;
        module_infos_[i].axis1.velocity=(position_tmp1-module_infos_[i].axis1.position)/read_update_dur.toSec();
        module_infos_[i].axis1.position=position_tmp1;
        if(module_infos_[i].axis1.has_torque_mode)
            module_infos_[i].axis1.effort=-1*trq_count1/module_infos_[i].axis1.count_Nm_factor;
        else
            module_infos_[i].axis1.effort=0;

        int32_t pos_count2=module_infos_[i].client_ptr->getAxis2PosCnt();
        int16_t trq_count2=module_infos_[i].client_ptr->getAxis2TrqCnt();
        int32_t pos_count_diff_2=pos_count2-module_infos_[i].axis2.count_zero;

        double position_tmp2=-1*pos_count_diff_2/module_infos_[i].axis2.count_rad_factor;
        module_infos_[i].axis2.velocity=(position_tmp2-module_infos_[i].axis2.position)/read_update_dur.toSec();
        module_infos_[i].axis2.position=position_tmp2;
        if(module_infos_[i].axis2.has_torque_mode)
            module_infos_[i].axis2.effort=-1*trq_count2/module_infos_[i].axis2.count_Nm_factor;
        else
            module_infos_[i].axis2.effort=0;
    }

}

void ElfinHWInterface::write_update()
{
    for(size_t i=0; i<module_infos_.size(); i++)
    {
        double position_cmd_count1=-1 * module_infos_[i].axis1.position_cmd * module_infos_[i].axis1.count_rad_factor + module_infos_[i].axis1.count_zero;
        double position_cmd_count2=-1 * module_infos_[i].axis2.position_cmd * module_infos_[i].axis2.count_rad_factor + module_infos_[i].axis2.count_zero;

        module_infos_[i].client_ptr->setAxis1PosCnt(int32_t(position_cmd_count1));
        module_infos_[i].client_ptr->setAxis2PosCnt(int32_t(position_cmd_count2));

        if(module_infos_[i].axis1.has_torque_mode)
        {
            double torque_cmd_count1=-1 * module_infos_[i].axis1.effort_cmd * module_infos_[i].axis1.count_Nm_factor;
            module_infos_[i].client_ptr->setAxis1TrqCnt(int16_t(torque_cmd_count1));
        }

        if(module_infos_[i].axis2.has_torque_mode)
        {
            double torque_cmd_count2=-1 * module_infos_[i].axis2.effort_cmd * module_infos_[i].axis2.count_Nm_factor;
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
