/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: memory.h
 * SystemC TLM2.0 component for simulating multiport memory
 * One tlm_socket master for Cpu
 * hw_Ssk tlm_sockets slave for hw accelerators
 */

#ifndef _MEMORY_H
#define _MEMORY_H

#include "systemc.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "models/monSystem/monSystem.h"

/*---
 * CLASS DEFINITION
 *----------------------------------------------------------------------------*/

class memory: public sc_core::sc_module, public ms::monSysSlave
{
  const uint8_t nb_HPx;
  const size_t size;
  const bool enable_dmi;
  uint8_t *mem;

public:
    tlm_utils::simple_target_socket_tagged<memory> cpu_Ssk;
    tlm_utils::simple_target_socket_tagged<memory> noc_Ssk;
    tlm_utils::simple_target_socket_tagged<memory> **HPx_Ssk;

    const sc_time LATENCY;

  memory(sc_core::sc_module_name name, uint8_t nbHPx, sc_time latency, size_t size_, bool enb_dmi);
    ~memory();

    virtual void b_transport(int id,
                             tlm::tlm_generic_payload& trans,
                             sc_time& delay);
    virtual bool get_direct_mem_ptr(int id,
                                    tlm::tlm_generic_payload& trans,
                                    tlm::tlm_dmi& dmi_data);
    virtual unsigned int transport_dbg(int id,
                                       tlm::tlm_generic_payload& trans);
};
#endif /*_MEMORY_H*/
