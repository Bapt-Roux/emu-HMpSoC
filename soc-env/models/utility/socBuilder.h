/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: socBuild.h
 * Configuration utility to build Soc from configuration file
 */

#ifndef _SOC_BUILDER
#define _SOC_BUILDER
#include "systemc.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "tlm_utils/tlm_quantumkeeper.h"

#include "socConfigStruct.h"
#include "zynq/xilinx-zynq.h"
#include "models/utility/scTrace.h"
#include "models/structural/iconnect.h"
#include "models/structural/scTime.h"
#include "models/structural/memory.h"
#include "models/structural/amba_GP.h"
#include "models/noc/nocIpc.h"
#include "models/noc/nocMono.h"
/* #include "models/monSystem/monSystem.h" */
#include "models/fpga/hwIP.h"
#include "models/fpga/machSuite-hwStub/machSuite_hws.h"
#include <fstream>

#include <sys/wait.h>


namespace sb{
  /*---
   * CLASS DEFINITION
   *----------------------------------------------------------------------------*/
  /*
   * ClusterBuilder class
   * Generate top level sc_module for one cluster of the SoC
   */
  class ClusterBuilder: public sc_core::sc_module
  {
    uint clusterId;
    clusterCnf clusterLevel;
    char* sk_descr;

    tlm_utils::simple_initiator_socket<ClusterBuilder> to_zynq_sk;
    tlm_utils::tlm_quantumkeeper m_qk;
    ms::monSysMaster *monitor;
    scTrace *tracer;
    iconnect *bus;
    scTime *getTime;
    memory *ram;
    amba_GP *amba_gp;
    hwIP **hw;

  public:
    SC_HAS_PROCESS(ClusterBuilder);
    ClusterBuilder(sc_core::sc_module_name name, uint id, generalCnf global, clusterCnf clusterConfig,
                   std::map<uint, noc::clPos>nocDiscover);
    ~ClusterBuilder();
    xilinx_zynq *cpu; //FIXME make it private
    noc::nocItf_base *iNoC;//FIXME make it private and write getter
    sc_signal<bool> rst;
  };

  /*
   * SoCBuilder class
   * Provides facilities to build SoC from textal configuration file
   * => Read configuration file, spawn cluster process
   * => After child process configuration start NoC router
   */
  class SoCBuilder
  {
    socCnf topLevel;
    std::map<uint,noc::clPos> nocDiscover;
    pid_t *clusters_pid;
    ClusterBuilder **clusters_sc;

  public:
    SoCBuilder(ifstream & stream, char stype);
    ~SoCBuilder();
    socCnf &getTopLevel(){return topLevel;};
    const std::map<uint,noc::clPos> getNocDiscover() const{ return nocDiscover;};

    void startNocRouter();
  };

}// end namespace sb
#endif /*_SOC_BUILDER*/
