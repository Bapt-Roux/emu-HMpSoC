/* ---
 *  This file define an API for easily manage the NoC
 *
 *  Author <baptiste.roux AT inria.fr>
 * -------------------------------------------------------------------------- */

#ifndef _NOC_API
#define _NOC_API
/*---
 * Basic principle:
 *   Master : send subscribe request to slave and manage message passing
 *            communication with them.
 *   Slaves : Wait for request from Master.
 * -------------------------------------------------------------------------- */

#include "noc_helper.h"
#include "cmd_noc.h"

#include <iostream>
#include<system_error>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stropts.h>
#include <string.h>
#include<vector>
#include<algorithm>
#include<utility>
#include<map>

extern "C"{
#include "monAlloc.h"
}
#ifdef NAPI_DEBUG
#   define print_DNAPI(message)                      \
  do {std::cout << __PRETTY_FUNCTION__<<": "<< message;}  \
  while (false)
#else
#   define print_DNAPI(message)                   \
  do {} while (false)
#endif

#define DFLT_NOC_DEVICE "/dev/scnoc"
#define MAX_CLUSTER 20

namespace napi {
  typedef std::pair<noc::task_t, uint64_t> trgtId_t;

   class nocAPI
  {
  protected:
    //device driver fd
    std::string deviceName;
    int noc_fd;
    //Utilities for noc use
    std::map<uint,noc::clPos> nocDiscover;
    uint nocLocalAddr;

    /* --- Low level functions:
     * Those functions directly interact with drivers                        --- */
    int8_t __discover();
    int8_t __register(noc::task_t task_id);
    int8_t __unRegister(noc::task_t task_id);
    int8_t __sendData(noc::orderHeader order);
    noc::orderHeader __waitOrder(noc::waitOrder wo);
    noc::ack_t __waitAck(noc::waitPoint wp);
    int8_t __getPowerLog(float &eng, float &time);

    /* --- Utility functions                                                   --- */
    uint getOffset(trgtId_t trgt);
    void sendAck(trgtId_t trgt, noc::ack_t ack_type);
    void memGet(trgtId_t trgt, uintptr_t lpAddr, size_t len);

  public:
    nocAPI(const std::string& device = DFLT_NOC_DEVICE);
    virtual ~nocAPI();

    /* --- Getter functions                                                  --- */
    const std::map<uint,noc::clPos> getClusters() const{ return nocDiscover;};
    const std::string getDeviceName() const{ return deviceName;};
    uint getClAddr() { return nocLocalAddr;};

    /*--- send & rcv data functions                                           --- */
    noc::task_t sendData(trgtId_t trgt, noc::task_t from, void* data, size_t data_length);
    int8_t b_sendData(trgtId_t trgt, noc::task_t from, void* data, size_t data_length);
    size_t b_rcvData(noc::task_t task_id, void* buffer, size_t maxSize);
    size_t b_rcvData(noc::task_t task_id, noc::task_t from_id, void* buffer, size_t maxSize);

    /*--- Sync functions                                             --- */
    int8_t ptp_syncMaster(trgtId_t trgt, noc::task_t from);
    int8_t ptp_syncReply(trgtId_t trgt, noc::task_t from);

  };

  /* ---
   * nocMaster: nocAPI interface to manage multiple woker dispatch over the HMpSoC
   --- */
  class nocMaster: public nocAPI
  {
    //status
    std::map<noc::task_t, uint64_t> linkedTasks;

  public:
    nocMaster(const std::string& device = DFLT_NOC_DEVICE);
    ~nocMaster();
    /*--- Registration and spawning functions                                 --- */
    int8_t registerTask(trgtId_t trgt);
    int8_t unregisterTask(noc::task_t task_id);

    /*--- Subscribe functions                                                 --- */
    int8_t askSlaveSubscribe(trgtId_t &trgt);
    std::vector<trgtId_t> brdSubscribe(uint reqWorker, uint8_t jobId);

    /*--- retrieve noc power  functions                                       --- */
    int8_t get_powerNoc(float& eng, float& time);

    /*--- Sync Barriers functions                                             --- */
    std::vector<noc::task_t> barrier_syncMaster(std::vector<noc::task_t> trgt, noc::task_t from);
  };

  /* ---
   * nocSlave: nocAPI interface acting as slave worker.
   --- */
  class nocSlave: public nocAPI
  {
    //status
    const trgtId_t master_id;

  public:
    nocSlave(trgtId_t mId, const std::string& device = DFLT_NOC_DEVICE);
    ~nocSlave();

    /* --- Getter functions                                                  --- */
    const trgtId_t getParent() const{ return master_id;};

    /*--- endPoint functions                                                   --- */
    noc::orderHeader ep_waitJob();
    void job_answer(trgtId_t trgt, noc::ack_t ack);

    /*--- Sync Barriers functions                                             --- */
    int8_t barrier_syncReply(trgtId_t trgt, noc::task_t from);
  };

}
#endif /*_NOC_API*/
