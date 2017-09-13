#include"memory.h"

/*---
 * METHODS IMPLEMENTATION
 *----------------------------------------------------------------------------*/
memory::memory(sc_core::sc_module_name name, uint8_t nbHPx, sc_time latency, size_t size_, bool enb_dmi)
  : sc_module(name), LATENCY(latency), nb_HPx(nbHPx), size(size_), monSysSlave(1+nbHPx), enable_dmi(enb_dmi),
    cpu_Ssk("cpu_Ssk"), noc_Ssk("noc_Ssk")
{
  //TODO check if socket id match channel parameters order
  cpu_Ssk.register_b_transport(this, &memory::b_transport, 0);
  cpu_Ssk.register_get_direct_mem_ptr(this, &memory::get_direct_mem_ptr, 0);
  cpu_Ssk.register_transport_dbg(this, &memory::transport_dbg, 0);

  noc_Ssk.register_b_transport(this, &memory::b_transport, 1);
  noc_Ssk.register_transport_dbg(this, &memory::transport_dbg, 1);

  HPx_Ssk = new tlm_utils::simple_target_socket_tagged<memory>*[nbHPx];
  char socket_Name[20];
  for (int sk=0 ; sk< nb_HPx; sk++){
    sprintf(socket_Name, "Hpx_%d", sk);
    HPx_Ssk[sk] = new tlm_utils::simple_target_socket_tagged<memory>(socket_Name);
    HPx_Ssk[sk]->register_b_transport(this, &memory::b_transport,sk+1);
    HPx_Ssk[sk]->register_get_direct_mem_ptr(this, &memory::get_direct_mem_ptr, sk+1);
    HPx_Ssk[sk]->register_transport_dbg(this, &memory::transport_dbg,sk+1);
  }
  mem = new uint8_t[size_];
  memset(&mem[0], 0, size); // init memory with 0x00

#ifdef MEMORY_DEBUG
  std::cout << name<<" Created " << std::endl;
#endif
}

memory::~memory()
{
  delete[] mem;
  for (int sk=0 ; sk< nb_HPx; sk++){
    delete HPx_Ssk[sk];
  }
  delete[] HPx_Ssk;
}

void memory::b_transport(int id, tlm::tlm_generic_payload& trans, sc_time& delay)
{
  tlm::tlm_command cmd = trans.get_command();
  sc_dt::uint64    addr = trans.get_address();
  unsigned char*   ptr = trans.get_data_ptr();
  unsigned int     len = trans.get_data_length();
  unsigned char*   byt = trans.get_byte_enable_ptr();

#ifdef MEMORY_DEBUG
    std::cout << sc_time_stamp() << "\t => " << name() << " : memory access @0x "<< hex << addr << std::endl;
#endif

  if (addr > sc_dt::uint64(size)) {
    trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    SC_REPORT_FATAL("Memory", "Unsupported access\n");
    return;
  }
  if (byt != 0) {
    trans.set_response_status(tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE);
    SC_REPORT_FATAL("Memory", "Unsupported access \n");
    return;
  }

  if (trans.get_command() == tlm::TLM_READ_COMMAND)
    memcpy(ptr, &mem[addr], len);
  else if (cmd == tlm::TLM_WRITE_COMMAND)
    memcpy(&mem[addr], ptr, len);

  //append communication cost
  delay += sc_time(comLog(len, id),SC_NS);

  trans.set_dmi_allowed(true);
  // trans.set_dmi_allowed(false);
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

bool memory::get_direct_mem_ptr(int id, tlm::tlm_generic_payload& trans,
                                tlm::tlm_dmi& dmi_data)
{
  if (enable_dmi){
    dmi_data.allow_read_write();
    dmi_data.set_dmi_ptr( reinterpret_cast<unsigned char*>(&mem[0]));
    dmi_data.set_start_address(0);
    dmi_data.set_end_address(size - 1);
    dmi_data.set_read_latency(LATENCY);
    dmi_data.set_write_latency(LATENCY);
    return true;
  }else{
    dmi_data.allow_none();
    return false;
  }
}

unsigned int memory::transport_dbg(int id, tlm::tlm_generic_payload& trans)
{
  tlm::tlm_command cmd = trans.get_command();
  sc_dt::uint64    addr = trans.get_address();
  unsigned char*   ptr = trans.get_data_ptr();
  unsigned int     len = trans.get_data_length();
  unsigned int num_bytes = (len < (size - addr)) ? len : (size - addr);
#ifdef MEMORY_DEBUG
  std::cout << sc_time_stamp() << "\t => " << name() << " : memory DBG access @0x "<< hex << addr << std::endl;
#endif
  if (cmd == tlm::TLM_READ_COMMAND)
    memcpy(ptr, &mem[addr], num_bytes);
  else if ( cmd == tlm::TLM_WRITE_COMMAND )
    memcpy(&mem[addr], ptr, num_bytes);

  return num_bytes;
}
