/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: monSystem.h
 * SystemC TLM2.0 component for configure and monitored virtual platform
 */

#ifndef _MON_SYSTEM
#define _MON_SYSTEM

#include "systemc.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"

#include "models/utility/cerealExt.h"
#include "msConfigStruct.h"
#include <fstream>
#include <cassert>

#define SLAVE_MEM_SIZE(nbChan) ((nbChan)*sizeof(ms::params) + 2*sizeof(float))
#define NS_TO_S (float)(0.000000001)
#define GLOBAL_REGSIZE 0x08

namespace ms{
  /*---
   * CLASS DEFINITION
   *----------------------------------------------------------------------------*/
  /*
   * Slave class
   * One tlm_socket slave for monSysMaster connection
   * provided a method for accumulate time and energy on the channel
   */
  class monSysSlave
  {
    const uint nb_chan;
    params *channels;
    float engCounter;
    float timeCounter;

  public:
    tlm_utils::simple_target_socket<monSysSlave> ms_Ssk;

    monSysSlave(uint nbChan);
    ~monSysSlave();
    virtual void ms_b_transport(tlm::tlm_generic_payload& trans,
                                sc_time& delay);
    uint comLog(size_t comSize, uint comChan);
  };

  /*
   * Master class
   * N_TARGET master socket for connection with monitoring IP
   *
   * A routing table enable the selection of the right output socket for a given addr
   */
    class monSysMaster: public sc_core::sc_module
  {
    const uint8_t nb_Msk;
    clusterProp cl_Props;
    ulong high_addr;
    std::map<sc_dt::uint64, uint> addrToSk;
    uint updtTransToSubspace(tlm::tlm_generic_payload &trans);
    void sendParamsTrans(uint blkAddr, uint chanNum, params *blkParams);
    float rtvGlobalLog(tlm::tlm_generic_payload &trans);

  public:
    tlm_utils::simple_target_socket<monSysMaster> cpu_Ssk;
    tlm_utils::simple_initiator_socket<monSysMaster> **ip_Msk;
    virtual void ms_b_transport(tlm::tlm_generic_payload& trans, sc_time& delay);
    void monSysInit(); //send default value to slave after construct
    ulong getAddrSpace();
    void exportClProp(std::ofstream &stream, char sType);//dump clusters property to stream

    monSysMaster(sc_core::sc_module_name name, uint8_t nbMsk, ms::clusterProp clProps);
    ~monSysMaster();
  };

};
#endif /*_MON_SYSTEM*/
