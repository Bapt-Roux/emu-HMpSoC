/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: nocIpc.h
 * Noc for inter systemC kernel communication
 */

#ifndef _NOC_BASE
#define _NOC_BASE
//nocCtrl primitives
#include "noc_helper.h"
//configuration struct
#include "models/utility/socConfigStruct.h"

// list of task and MS calculation
#include <math.h>
#include <vector>

//systemC header
#include "systemc.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "tlm_utils/tlm_quantumkeeper.h"

#ifndef NDEBUG
#   define ASSERT(condition, message)                                   \
  do {                                                                  \
    if (! (condition)) {                                                \
      std::cerr << "Assertion `" #condition "` failed in " << __FILE__  \
                << " line " << __LINE__ << ": " << message << std::endl; \
      std::terminate();                                                 \
    }                                                                   \
  } while (false)

#ifdef NOC_DEBUG
#define NOC_DEBUG_BIND
#define NOC_DEBUG_LOG
#endif

#else
#   define ASSERT(condition, message) do { } while (false)
#endif

//register getter/setter
#define _RNOC(NAME) (((uintptr_t)noc_register+ (RNOC_##NAME)))

namespace noc{
  class nocItf_base: public sc_core::sc_module
  {
  protected:
    virtual void b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)=0;

    //NoC job management
    std::map<task_t, uint> pendingOrder;
    std::map<task_t, uint> pendingWait;
    std::vector<waitPoint> wakeupTask;
    std::vector<orderHeader> rcvOrder;

    //NoC discover
    std::map<uint,noc::clPos> nocDiscover;

    //NoC ctrl buffer
    uint8_t *noc_register;
    virtual void cpuCmdEvent()=0;

    //PowerLog buffer
    float enj_noc;
    uint tns_noc;

  public:
    /* irq lines */
    sc_out<bool> irq_wakeUp;
    sc_out<bool> irq_orderRcv;

    /* tlm sockets */
    tlm_utils::simple_target_socket<nocItf_base> cmd_Ssk;
    tlm_utils::simple_initiator_socket<nocItf_base> DMA_Msk;

    nocItf_base(sc_core::sc_module_name name, std::map<uint, noc::clPos>nocDiscover);
    ~nocItf_base();

    //nocSk getter
    virtual  tlm_utils::simple_target_socket<nocItf_base>* get_noc_Ssk(){return NULL;};
    virtual  tlm_utils::simple_initiator_socket<nocItf_base>* get_noc_Msk(){return NULL;};

    //FIXME: change encapsulation if possible: this function only relevant for IPC impl
    virtual void openUnixIpc(std::string socketPath){}
    virtual void closeUnixIpc(){}
    virtual void syncUnixMsg(){}
  };

  class nocRouter_base
  {
  protected:
    std::map<uint64_t,uint> clMapOnSk;
    std::map<uint,clPos> skToClpos;
    uint nb_sk;
    uint toClusterAddr(uint64_t &addr);

    //Monitor Systems vars
    sb::nocCnf nocConfig;
    void comLog(uint fromSk, uint toSk, size_t size);
    uint nHop(clPos from, clPos to);
    float enj_log;
    uint tns_log;

  public:
    nocRouter_base(int nbSk, sb::nocCnf config);
    ~nocRouter_base(){}
    virtual uint bindToCluster(tlm_utils::simple_target_socket<nocItf_base>* nocItf_Ssk,
                               tlm_utils::simple_initiator_socket<nocItf_base>* nocItf_Msk,
                               uint64_t clBaseAddr, uint8_t cl_x, uint8_t cl_y)=0;
    //FIXME: change encapsulation if possible: this function only relevant for IPC impl
    virtual void startRouter(){}
  };
}// end namespace noc
#endif /*_NOC_BASE*/
