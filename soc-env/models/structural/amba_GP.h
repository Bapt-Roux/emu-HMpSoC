/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: amba_GP.h
 * SystemC TLM2.0 component for simulating amba general purpose memory map bus
 * One tlm_socket master for Cpu
 * nb_Msk tlm_socket slave for hw accelerators
 */
#ifndef _AMBA_GP
#define _AMBA_GP

#include "systemc.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "models/monSystem/monSystem.h"
#include "models/utility/socConfigStruct.h"

#define FW_DELAY sc_time(10,SC_NS)

/*---
 * CLASS DEFINITION
 *----------------------------------------------------------------------------*/
class amba_GP: public sc_core::sc_module, public ms::monSysSlave
{
  const uint8_t nb_Msk;
  uint8_t *amba_mem;
  size_t size;
  std::map<sc_dt::uint64, uint> addrToSk;
  uint updtTransToSubspace(tlm::tlm_generic_payload &trans);

 public:
  tlm_utils::simple_target_socket<amba_GP> cpu_Ssk;
  tlm_utils::simple_initiator_socket<amba_GP> **pl_Msk;

  amba_GP(sc_core::sc_module_name name, uint8_t nbMsk, cereal::serialMap<sb::hwCpnCnf> hwCpn);
  ~amba_GP();

  virtual void b_transport(tlm::tlm_generic_payload& trans,
                           sc_time& delay);
};

#endif /*_AMBA_GP*/
