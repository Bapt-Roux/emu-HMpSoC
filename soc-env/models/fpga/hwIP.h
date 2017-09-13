/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: ipHw.h
 * SystemC core stub for hardware ip
 */
#ifndef _HWIP_H
#define _HWIP_H

#include "systemc.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "models/monSystem/monSystem.h"

/*---
 * COMMAND/STATUS/REG_OFFSET BASE DEFINITION
 *----------------------------------------------------------------------------*/
#define REG_CMD			0x00
#define REG_STATUS		0x01
#define REG_PARAMS    0x04
#define REG_ARRAY_IN  0x20
#define REG_ARRAY_OUT 0x30
#define GENIP_RPSIZE	(uint8_t)0x40

#define CMD_START 0x80
//others CMD bits are used for reset irqLine
#define CMD_RST_IRQ(n) (0x1<<(n))

#define STATUS_DONE 0x80
//others STATUS bits are used for irqLine
#define STATUS_IRQ(n) (0x1<<(n))

//register getter/setter
#define _REG(NAME) (register_pool+ (REG_##NAME))

/*---
 * CLASS DEFINITION
 *----------------------------------------------------------------------------*/
class hwIP: public sc_core::sc_module, public ms::monSysSlave
{
  const uint8_t nbDma_Msk;
  const uint8_t nbHw_Irq;
  const uint8_t reg_poolSize;

 protected:
  sc_event cmd_events;
  //register pool access through maccro
  uint8_t *register_pool;
  void memRead(uint32_t addr, void* buffer, size_t len, int line_DMA);
    void memWrite(uint32_t addr, void* buffer, size_t len, int line_DMA);
  virtual void parse_cmd(){};

 public:
  // cnf/ctrl slave_socket
  tlm_utils::simple_target_socket<hwIP> amba_Ssk;
  //memory ports
  tlm_utils::simple_initiator_socket_tagged<hwIP> **dma_Msk;
  // irq ports
  sc_out<bool> *irqn;  // Interrupt connection

  //hw ports
  virtual void b_amba(tlm::tlm_generic_payload& trans,
                           sc_time& delay);

  hwIP(sc_core::sc_module_name name,uint8_t nbDmaMsk, uint8_t nbIrq, uint8_t regPoolSize);
  virtual ~hwIP();
};

#endif /*_IPHW_H*/
