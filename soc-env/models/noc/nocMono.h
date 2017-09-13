/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: nocMono.h
 * Noc for intercluster communication with only one systemC simulation kernel
 */

#ifndef _NOC_MONO
#define _NOC_MONO
#include "nocBase.h"

//Debug FLAGS management
#ifdef NMONO_DEBUG
#define NMONO_DEBUG_TLM //TLM layer
#define NMONO_DEBUG_BIND //Cluster Binding mechanism
#define NMONO_DEBUG_CPU  //Storing and Ack request mechanisms
#endif

namespace noc{
  namespace mono{
    /*
     * nocItf class
     * Connection point for cluster
     */
    class nocItf: public noc::nocItf_base
    {
      void b_transport(tlm::tlm_generic_payload& trans, sc_time& delay);
      void b_noc_transport(tlm::tlm_generic_payload& trans, sc_time& delay);
      void cpuCmdEvent();
      tlm_utils::simple_target_socket<nocItf> nocItf_Ssk;
      tlm_utils::simple_initiator_socket<nocItf> nocItf_Msk;

    public:
      //nocSk getter
      tlm_utils::simple_target_socket<nocItf_base>* get_noc_Ssk();
      tlm_utils::simple_initiator_socket<nocItf_base>* get_noc_Msk();

      nocItf(sc_core::sc_module_name name, std::map<uint, noc::clPos>nocDiscover);
      ~nocItf();
    };

    class nocRouter: public noc::nocRouter_base, public sc_module
    {
      virtual void b_transport(int id, tlm::tlm_generic_payload& trans, sc_time& delay);

    public:
      nocRouter(sc_core::sc_module_name name, int nbSk, sb::nocCnf config);
      ~nocRouter();
      tlm_utils::simple_target_socket_tagged<nocRouter>** noc_Ssk;
      tlm_utils::simple_initiator_socket<nocRouter>** noc_Msk;
      uint bindToCluster(tlm_utils::simple_target_socket<nocItf_base>* nocItf_Ssk,
                                 tlm_utils::simple_initiator_socket<nocItf_base>* nocItf_Msk,
                         uint64_t clBaseAddr, uint8_t cl_x, uint8_t cl_y);
    };

  }// end namespace mono
}// end namespace noc
#endif /*_NOC_MONO*/
