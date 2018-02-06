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

#ifndef ELFIN_ETHERCAT_MANAGER_H
#define ELFIN_ETHERCAT_MANAGER_H

#include <stdexcept>
#include <string>

#include <stdint.h>

#include <boost/scoped_array.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace elfin_ethercat_driver {

typedef struct {
    std::string name;
    int32_t value;
    uint8_t channel;
}ElfinPDOunit;

/**
 * \brief EtherCAT exception. Currently this is only thrown in the event
 *        of a failure to construct an EtherCat manager.
 */
class EtherCatError : public std::runtime_error
{
public:
  explicit EtherCatError(const std::string& what)
    : std::runtime_error(what)
  {}
};

/**
 * \brief This class provides a CPP interface to the SimpleOpenEthercatMaster library
 * Given the name of an ethernet device, such as "eth0", it will connect,
 * start a thread that cycles data around the network, and provide read/write
 * access to the underlying io map.
 *
 * Please note that as used in these docs, 'Input' and 'Output' are relative to
 * your program. So the 'Output' registers are the ones you write to, for example.
 */
class EtherCatManager
{
public:
  /**
   * \brief Constructs and initializes the ethercat slaves on a given network interface.
   *
   * @param[in] ifname the name of the network interface that the ethercat chain 
   *                   is connected to (i.e. "eth0")
   *
   * Constructor can throw EtherCatError exception if SOEM could not be 
   * initialized.
   */
  EtherCatManager(const std::string& ifname);
  
  ~EtherCatManager();

  /**
   * \brief writes 'value' to the 'channel-th' output-register of the given 'slave'
   *  
   * @param[in] slave_no The slave number of the device to write to (>= 1)
   * @param[in] channel The byte offset into the output IOMap to write value to
   * @param[in] value The byte value to write
   *
   * This method currently makes no attempt to catch out of bounds errors. Make
   * sure you know your IOMap bounds.
   */
  void write(int slave_no, uint8_t channel, uint8_t value);

  /**
   * \brief Reads the "channel-th" input-register of the given slave no
   *  
   * @param[in] slave_no The slave number of the device to read from (>= 1)
   * @param[in] channel The byte offset into the input IOMap to read from
   */
  uint8_t readInput(int slave_no, uint8_t channel) const;

  /**
   * \brief Reads the "channel-th" output-register of the given slave no
   *  
   * @param[in] slave_no The slave number of the device to read from (>= 1)
   * @param[in] channel The byte offset into the output IOMap to read from
   */
  uint8_t readOutput(int slave_no, uint8_t channel) const;

  /**
   * \brief write the SDO object of the given slave no
   *
   * @param[in] slave_no The slave number of the device to read from (>= 1)
   * @param[in] index The index address of the parameter in SDO object
   * @param[in] subidx The sub-index address of the parameter in SDO object
   * @param[in] value value to write
   */
  template <typename T>
  uint8_t writeSDO(int slave_no, uint16_t index, uint8_t subidx, T value) const;

  /**
   * \brief read the SDO object of the given slave no
   *
   * @param[in] slave_no The slave number of the device to read from (>= 1)
   * @param[in] index The index address of the parameter in SDO object
   * @param[in] subidx The sub-index address of the parameter in SDO object
   */
  template <typename T>
  T readSDO(int slave_no, uint16_t index, uint8_t subidx) const;

  /**
   * \brief get the number of clients
   */
  int getNumClinets() const;

private:
  bool initSoem(const std::string& ifname);

  const std::string ifname_;
  uint8_t iomap_[4096];
  int num_clients_;
  boost::thread cycle_thread_;
  mutable boost::mutex iomap_mutex_;
  bool stop_flag_;
};

}

#endif

