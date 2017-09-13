/*
 * Copyright (c) 2017 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * SystemC cosimulation manager
 * Enable qemu device to read simulation time and to stop the systemC simulation
 * kernel.
 */

#ifndef _SC_TIME
#define _SC_TIME

#include "systemc.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"

#define SCTIME_GTIME_NS 0x00
#define SCTIME_GCLOCK   0x08
#define SCTIME_RSIZE    0x0f


#define SEC_TO_NS (1000000000)

/*---
 * CLASS DEFINITION
 *----------------------------------------------------------------------------*/
class scTime: public sc_core::sc_module
{
public:
  tlm_utils::simple_target_socket<scTime> time_Ssk;
  scTime(sc_core::sc_module_name name);
  ~scTime();

  virtual void b_transport(tlm::tlm_generic_payload& trans,
                           sc_time& delay);
};

#endif /*_SC_TIME*/
