/*
 * Copyright (C) 2015, Jonathan Meyer
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Tokyo Opensource Robotics Kyokai Association. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// copied from https://github.com/ros-industrial/robotiq/blob/jade-devel/robotiq_ethercat/src/ethercat_manager.cpp


#include "elfin_ethercat_driver/elfin_ethercat_manager.h"

#include <unistd.h>
#include <stdio.h>
#include <time.h>

#include <boost/ref.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>

namespace 
{
static const unsigned THREAD_SLEEP_TIME = 1000; // 1 ms
static const unsigned EC_TIMEOUTMON = 500;
static const int NSEC_PER_SECOND = 1e+9;
void timespecInc(struct timespec &tick, int nsec)
{
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= NSEC_PER_SECOND)
    {
      tick.tv_nsec -= NSEC_PER_SECOND;
      tick.tv_sec++;
    }
}

void handleErrors()
{
  /* one ore more slaves are not responding */
  ec_group[0].docheckstate = FALSE;
  ec_readstate();
  for (int slave = 1; slave <= ec_slavecount; slave++)
  {
    if ((ec_slave[slave].group == 0) && (ec_slave[slave].state != EC_STATE_OPERATIONAL) && slave != 4)
    {
      ec_group[0].docheckstate = TRUE;
      if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
      {
        fprintf(stderr, "ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
        ec_writestate(slave);
      }
      else if(ec_slave[slave].state == EC_STATE_SAFE_OP && slave != 4)
      {
        fprintf(stderr, "WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
        ec_slave[slave].state = EC_STATE_OPERATIONAL;
        ec_writestate(slave);
      }
      else if(slave == 4 && ec_slave[slave].state != EC_STATE_SAFE_OP)
      {
        fprintf(stderr, "WARNING : slave %d is no in SAFE_OP, change to SAFE_OP.\n", slave);
        ec_slave[slave].state = EC_STATE_SAFE_OP;
        ec_writestate(slave);
      }
      else if(ec_slave[slave].state > 0)
      {
        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
        {
          ec_slave[slave].islost = FALSE;
          printf("MESSAGE : slave %d reconfigured\n",slave);
        }
      }
      else if(!ec_slave[slave].islost)
      {
        /* re-check state */
        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
        if (!ec_slave[slave].state)
        {
          ec_slave[slave].islost = TRUE;
          fprintf(stderr, "ERROR : slave %d lost\n",slave);
        }
      }
    }
    if (ec_slave[slave].islost)
    {
      if(!ec_slave[slave].state)
      {
        if (ec_recover_slave(slave, EC_TIMEOUTMON))
        {
          ec_slave[slave].islost = FALSE;
          printf("MESSAGE : slave %d recovered\n",slave);
        }
      }
      else
      {
        ec_slave[slave].islost = FALSE;
        printf("MESSAGE : slave %d found\n",slave);
      }
    }
  }
}

void cycleWorker(boost::mutex& mutex, bool& stop_flag)
{
  // 1ms in nanoseconds
  double period = THREAD_SLEEP_TIME * 1000;
  // get current time
  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);
  timespecInc(tick, period);
  // time for checking overrun
  struct timespec before;
  double overrun_time;
  while (!stop_flag) 
  {
    int expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    int sent, wkc;
    {
      boost::mutex::scoped_lock lock(mutex);
      sent = ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
    }

    if (wkc < expected_wkc)
    {
      handleErrors();
    }

    // check overrun
    clock_gettime(CLOCK_REALTIME, &before);
    overrun_time = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) -  (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);
    if (overrun_time > 0.0)
    {
      tick.tv_sec=before.tv_sec;
      tick.tv_nsec=before.tv_nsec;
    }
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    timespecInc(tick, period);
  }
}

} // end of anonymous namespace


namespace elfin_ethercat_driver {

EtherCatManager::EtherCatManager(const std::string& ifname)
  : ifname_(ifname), 
    num_clients_(0),
    stop_flag_(false)
{
  // initialize iomap
  for(int i=0; i<4096; i++)
  {
    iomap_[i]=0;
  }

  if (initSoem(ifname)) 
  {
    cycle_thread_ = boost::thread(cycleWorker, 
                                  boost::ref(iomap_mutex_),
                                  boost::ref(stop_flag_));
  } 
  else 
  {
    // construction failed
    throw EtherCatError("Could not initialize SOEM");
  }
}

EtherCatManager::~EtherCatManager()
{
  stop_flag_ = true;

  // Request init operational state for all slaves
  ec_slave[0].state = EC_STATE_INIT;

  /* request init state for all slaves */
  ec_writestate(0);

  //stop SOEM, close socket
  ec_close();
  cycle_thread_.join();
}

bool EtherCatManager::initSoem(const std::string& ifname) {
  // Copy string contents because SOEM library doesn't 
  // practice const correctness
  const static unsigned MAX_BUFF_SIZE = 1024;
  char buffer[MAX_BUFF_SIZE];
  size_t name_size = ifname_.size();
  if (name_size > sizeof(buffer) - 1) 
  {
    fprintf(stderr, "Ifname %s exceeds maximum size of %u bytes\n", ifname_.c_str(), MAX_BUFF_SIZE);
    return false;
  }
  std::strncpy(buffer, ifname_.c_str(), MAX_BUFF_SIZE);

  printf("Initializing etherCAT master\n");

  if (!ec_init(buffer))
  {
    fprintf(stderr, "Could not initialize ethercat driver\n");
    return false;
  }

  /* find and auto-config slaves */
  if (ec_config_init(FALSE) <= 0)
  {
    fprintf(stderr, "No slaves are found on %s\n", ifname_.c_str());
    return false;
  }

  printf("SOEM found and configured %d slaves\n", ec_slavecount);

  if (ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_PRE_OP)
  {
    fprintf(stderr, "Could not set EC_STATE_PRE_OP\n");
    return false;
  }

  // configure IOMap
  int iomap_size = ec_config_map(iomap_);
  printf("SOEM IOMap size: %d\n", iomap_size);

  // locates dc slaves - ???
  ec_configdc();

  // '0' here addresses all slaves
  if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_SAFE_OP)
  {
    fprintf(stderr, "Could not set EC_STATE_SAFE_OP\n");
    return false;
  }

  /* 
      This section attempts to bring all slaves to operational status. It does so
      by attempting to set the status of all slaves (ec_slave[0]) to operational,
      then proceeding through 40 send/recieve cycles each waiting up to 50 ms for a
      response about the status. 
  */
  for(int i=1;i<4;i++){
    ec_slave[i].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    ec_writestate(i);
    int chk = 40;
    do {
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_statecheck(i, EC_STATE_OPERATIONAL, 50000); // 50 ms wait for state check
    } while (chk-- && (ec_slave[i].state != EC_STATE_OPERATIONAL));

    if(ec_statecheck(i,EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
    {
      fprintf(stderr, "OPERATIONAL state not set, exiting\n");
      return false;
    }
  }
  ec_readstate();

  printf("\nFinished configuration successfully\n");
  return true;
}

int EtherCatManager::getNumClinets() const
{
  return num_clients_;
}

void EtherCatManager::write(int slave_no, uint8_t channel, uint8_t value)
{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  if (slave_no > ec_slavecount) {
    fprintf(stderr, "ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  if (channel*8 >= ec_slave[slave_no].Obits) {
    fprintf(stderr, "ERROR : slave_no(%d) : channel(%d) is larger than Output bits (%d), you may need to read elfin_robot/docs/Fix_ESI.md or elfin_robot/docs/Fix_ESI_english.md with a Markdown editor or on github.com\n", slave_no, channel*8, ec_slave[slave_no].Obits);
    exit(1);
  }
  ec_slave[slave_no].outputs[channel] = value;
}

uint8_t EtherCatManager::readInput(int slave_no, uint8_t channel) const
{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  if (slave_no > ec_slavecount) {
    fprintf(stderr, "ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  if (channel*8 >= ec_slave[slave_no].Ibits) {
    fprintf(stderr, "ERROR : slave_no(%d) : channel(%d) is larger than Input bits (%d), you may need to read elfin_robot/docs/Fix_ESI.md or elfin_robot/docs/Fix_ESI_english.md with a Markdown editor or on github.com\n", slave_no, channel*8, ec_slave[slave_no].Ibits);
    exit(1);
  }
  return ec_slave[slave_no].inputs[channel];
}

uint8_t EtherCatManager::readOutput(int slave_no, uint8_t channel) const
{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  if (slave_no > ec_slavecount) {
    fprintf(stderr, "ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  if (channel*8 >= ec_slave[slave_no].Obits) {
    fprintf(stderr, "ERROR : slave_no(%d) : channel(%d) is larger than Output bits (%d), you may need to read elfin_robot/docs/Fix_ESI.md or elfin_robot/docs/Fix_ESI_english.md with a Markdown editor or on github.com\n", slave_no, channel*8, ec_slave[slave_no].Obits);
    exit(1);
  }
  return ec_slave[slave_no].outputs[channel];
}

template <typename T>
uint8_t EtherCatManager::writeSDO(int slave_no, uint16_t index, uint8_t subidx, T value) const
{
  int ret;
  ret = ec_SDOwrite(slave_no, index, subidx, FALSE, sizeof(value), &value, EC_TIMEOUTSAFE);
  return ret;
}

template <typename T>
T EtherCatManager::readSDO(int slave_no, uint16_t index, uint8_t subidx) const
{
  int ret, l;
  T val;
  l = sizeof(val);
  ret = ec_SDOread(slave_no, index, subidx, FALSE, &l, &val, EC_TIMEOUTRXM);
  if ( ret <= 0 ) { // ret = Workcounter from last slave response
    fprintf(stderr, "Failed to read from ret:%d, slave_no:%d, index:0x%04x, subidx:0x%02x\n", ret, slave_no, index, subidx);
  }
  return val;
}

template uint8_t EtherCatManager::writeSDO<char> (int slave_no, uint16_t index, uint8_t subidx, char value) const;
template uint8_t EtherCatManager::writeSDO<int> (int slave_no, uint16_t index, uint8_t subidx, int value) const;
template uint8_t EtherCatManager::writeSDO<int8_t> (int slave_no, uint16_t index, uint8_t subidx, int8_t value) const;
template uint8_t EtherCatManager::writeSDO<short> (int slave_no, uint16_t index, uint8_t subidx, short value) const;
template uint8_t EtherCatManager::writeSDO<long> (int slave_no, uint16_t index, uint8_t subidx, long value) const;
template uint8_t EtherCatManager::writeSDO<unsigned char> (int slave_no, uint16_t index, uint8_t subidx, unsigned char value) const;
template uint8_t EtherCatManager::writeSDO<unsigned int> (int slave_no, uint16_t index, uint8_t subidx, unsigned int value) const;
template uint8_t EtherCatManager::writeSDO<unsigned short> (int slave_no, uint16_t index, uint8_t subidx, unsigned short value) const;
template uint8_t EtherCatManager::writeSDO<unsigned long> (int slave_no, uint16_t index, uint8_t subidx, unsigned long value) const;

template char EtherCatManager::readSDO<char> (int slave_no, uint16_t index, uint8_t subidx) const;
template int EtherCatManager::readSDO<int> (int slave_no, uint16_t index, uint8_t subidx) const;
template short EtherCatManager::readSDO<short> (int slave_no, uint16_t index, uint8_t subidx) const;
template long EtherCatManager::readSDO<long> (int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned char EtherCatManager::readSDO<unsigned char> (int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned int EtherCatManager::readSDO<unsigned int> (int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned short EtherCatManager::readSDO<unsigned short> (int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned long EtherCatManager::readSDO<unsigned long> (int slave_no, uint16_t index, uint8_t subidx) const;

}

