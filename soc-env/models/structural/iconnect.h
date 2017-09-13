/*
 * Simple TLM model of an interconnect
 *
 * Copyright (c) 2011 Edgar E. Iglesias.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * To differentiate between targets that want to be passed absolute
 * addresses with every transaction. Most targets or slaves will use
 * the relative mode. But for example, when bridging accesses over to
 * any slave inside a QEMU instance, the addresses need to be absolute.
 */

#ifndef _ICONNECT_H
#define _ICONNECT_H

#include "systemc.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"

enum addrmode {
  ADDRMODE_RELATIVE,
  ADDRMODE_ABSOLUTE
};

struct memmap_entry {
  uint64_t addr;
  uint64_t size;
  enum addrmode addrmode;
  int sk_idx;
};

/*---
 * CLASS DEFINITION
 *----------------------------------------------------------------------------*/
class iconnect : public sc_core::sc_module
{
  const uint8_t nb_Msk;
  const uint8_t nb_Ssk;
  unsigned int map_address(sc_dt::uint64 addr, sc_dt::uint64& offset);
  void unmap_offset(unsigned int target_nr,
                    sc_dt::uint64 offset, sc_dt::uint64& addr);
 public:
  tlm_utils::simple_target_socket_tagged<iconnect> **cpu_Ssk;
  tlm_utils::simple_initiator_socket_tagged<iconnect> **ic_Msk;
  memmap_entry *map;

  SC_HAS_PROCESS(iconnect);
  iconnect(sc_core::sc_module_name name, uint8_t nbSsk, uint8_t nb_Msk);
  ~iconnect();
  virtual void b_transport(int id,
                           tlm::tlm_generic_payload& trans,
                           sc_time& delay);

  virtual bool get_direct_mem_ptr(int id,
                                  tlm::tlm_generic_payload& trans,
                                  tlm::tlm_dmi&  dmi_data);

  virtual unsigned int transport_dbg(int id,
                                     tlm::tlm_generic_payload& trans);

  virtual void invalidate_direct_mem_ptr(int id,
                                         sc_dt::uint64 start_range,
                                         sc_dt::uint64 end_range);


	int memmap(sc_dt::uint64 addr, sc_dt::uint64 size,
             enum addrmode addrmode, int idx, tlm::tlm_target_socket<> &s);
};
#endif /*_ICONNECT_H*/
