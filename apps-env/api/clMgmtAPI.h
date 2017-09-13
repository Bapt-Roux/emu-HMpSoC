/* ---
 *  This file define an API for working thread in clusters
 *
 *  Author <baptiste.roux AT inria.fr>
 * -------------------------------------------------------------------------- */

#ifndef _CLMGMT_API
#define _CLMGMT_API

#include <iostream>
#include<system_error>
#include <sys/wait.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include<deque>
#include<utility>
#include<map>

#include"nocAPI.h"
#include "machSuite-callStub/machSuite_cls.h"

// Use for local router mgmt
#include<pthread.h>
#include<semaphore.h>

extern "C"{
#include"monAlloc.h"
}

#include <stropts.h>
#include <poll.h>

#ifdef CLMGMT_DEBUG
#   define print_DMGMT(message)                      \
  do {std::cout << __PRETTY_FUNCTION__<<": "<< message;}  \
  while (false)
#else
#   define print_DMGMT(message)                   \
  do {} while (false)
#endif

#define PIPE_TIMEOUT_MS 500
#define MAX_PROVIDERS 8
#define MAX_RECEIVER 8

namespace clmgmt {
  // intra-Cluster communications header
  typedef struct{
    noc::task_t trgt;
    uintptr_t blk_head;
    size_t offset;
    size_t size;
  }localHeader_t;

  typedef struct{
    pid_t pid;
    int wPipe;
    int rPipe;
  }subPrc_t;

  //Providers and receiver structures
  typedef struct{
    noc::task_t task;
    uint64_t clAddr;
    size_t dSize;
  }comProps;

  //worker environment structure
  typedef struct{
    int *wPipe;
    int *rPipe;
    std::string nocDevice;
    napi::trgtId_t parent;
  }worker_env;

  // intra-Cluster communications functions
  struct icCom{
    static int8_t lMemGive(subPrc_t comItf, noc::task_t task_id, void *mem, size_t size);
    static void *b_lMemAcquire(subPrc_t comItf, noc::task_t task_id, size_t &size);
  };

  // intra-Cluster communications router
  class icRouter{
    const uint slavesTrigger;
    pthread_mutex_t lock;
    pthread_cond_t event_sig;
    bool kill;
    std::map<noc::task_t, subPrc_t> childPool;

  public:
    pthread_t pid;
    icRouter(uint nbTrigger);
    ~icRouter();
    void addChild(noc::task_t tsk, subPrc_t sPrc);
    void routerWaitRun();
    void updtChild();
    void stopRouter();
    subPrc_t getSubPrcOf(noc::task_t);
    static void routeMsg(void *args);
  };

  class clMBase
  {
  private:
    const std::string nocdev;

  protected:
    icRouter localRouter;
    std::vector<pid_t> spawnedChild;

  public:
    clMBase(std::string dev, uint routerTrigger);
    ~clMBase();
    int8_t spawnChild(napi::trgtId_t parent);
    static void resetCPM();
    static void getCPM(float &eng, float &time);
  };

  class clMaster: public clMBase
  {
    napi::nocMaster nocApi;
    float sNocEng, sNocTime;
  public:
    clMaster(const std::string& device = DFLT_NOC_DEVICE);
    ~clMaster();
    napi::nocMaster& getNocApi(){return nocApi;}

    int8_t askSlaveSubscribe(napi::trgtId_t &trgt);

    /*--- Data xfer initialization for spawned trgt                            --- */
    void setProviderAndReceiver(napi::trgtId_t target, std::deque<comProps> providers,
                                std::deque<comProps> receivers);
    /*--- Data xfer wrapper around local and noc communications --- */
    int8_t b_sendData(napi::trgtId_t trgt, void* data, size_t data_length);
    size_t b_rcvData(napi::trgtId_t trgt, void* &buffer, size_t maxSize);
  };

  class clEndPoint: public clMBase
  {
    napi::nocSlave nocApi;
  public:
    clEndPoint(const std::string& device = DFLT_NOC_DEVICE);
    ~clEndPoint();
    void nocEndPoint();
  };

  class clSlave
  {
    //Noc interface
    napi::nocSlave nocApi;

    // id and parent
    noc::task_t slave_id;
    subPrc_t masterPrc;

    //Providers and receiver properties
    std::deque<comProps> providersList;
    std::deque<comProps> receiversList;

    /*--- Data xfer initialization                                             --- */
    void getProviderAndReceiver();

    /*--- Worker setup and start                                               --- */
    void startWorker();
    void startSamplesWorker();

    public:
    clSlave(worker_env env);
    ~clSlave();
  };

}
#endif /*_CLMGMT_API*/
