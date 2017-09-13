#include"hwIP.h"

/*---
 * METHODS IMPLEMENTATION
 *----------------------------------------------------------------------------*/
hwIP::hwIP(sc_core::sc_module_name name,uint8_t nbDmaMsk, uint8_t nbIrq, uint8_t regPoolSize)
  :sc_module(name), nbDma_Msk(nbDmaMsk), nbHw_Irq(nbIrq), reg_poolSize(regPoolSize),
   monSysSlave(1), amba_Ssk("amba_Ssk")
{
  // init syctemC socket
  amba_Ssk.register_b_transport(this, &hwIP::b_amba);

  char txt[32];
  dma_Msk = new tlm_utils::simple_initiator_socket_tagged<hwIP>*[nbDmaMsk];
  for (uint sk = 0; sk < nbDmaMsk; sk++) {
    sprintf(txt, "dma_%d", sk);
    dma_Msk[sk] = new tlm_utils::simple_initiator_socket_tagged<hwIP>(txt);
  }

  //init Irq channel
  irqn = new sc_out<bool>[nbIrq];

  //allocate register memory
  register_pool = new uint8_t[regPoolSize];
  memset(register_pool, 0x00, sizeof(uint8_t)*reg_poolSize);
}

hwIP::~hwIP(){
  //DMA sockets
  for (uint sk=0 ; sk< nbDma_Msk; sk++){
    delete dma_Msk[sk];
  }
  delete[] dma_Msk;
  //Irq sockets
  delete[] irqn;
  //delete register pool
  delete[] register_pool;
}

void hwIP::b_amba(tlm::tlm_generic_payload& trans, sc_time& delay)
{
#ifdef HWIP_DEBUG
  std::cout << "@ "<< sc_time_stamp() << " ==>"<< "HWIP received Transaction" << std::endl;
  std::cout << "\t addr:"<< trans.get_address()<< std::endl;
  std::cout << "\t CMD:"<< trans.get_command() << std::endl;
  std::cout << "\t length:"<< trans.get_data_length() << std::endl;
#endif

  tlm::tlm_command cmd = trans.get_command();
  uint32_t    addr		= trans.get_address();
  unsigned char*   ptr	= trans.get_data_ptr();
  unsigned int     len	= trans.get_data_length();
  unsigned char*   byt	= trans.get_byte_enable_ptr();

  if (reg_poolSize < addr) {
    trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    SC_REPORT_FATAL("hwIP", "Register Acess failed out of range\n");
    return;
  }
  if (byt != 0) {
    trans.set_response_status(tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE);
    SC_REPORT_FATAL("hwIP", "Unsupported access \n");
    return;
  }

  if (trans.get_command() == tlm::TLM_READ_COMMAND){
    memcpy(ptr, &register_pool[addr], len);
  }
  else if (cmd == tlm::TLM_WRITE_COMMAND){
    memcpy(&register_pool[addr], ptr, len);
    if ( REG_CMD == trans.get_address()) { //wake up parse_cmd Thread
      cmd_events.notify();
    }
  }

  /*
   * Append communication to monitor system and update message cost
   */
  uint tmp = comLog(trans.get_data_length(), 0);
  delay += sc_time(tmp,SC_NS);

  trans.set_dmi_allowed(false);
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

void hwIP::memRead(uint32_t addr, void* buffer, size_t len, int line_DMA)
{
  tlm::tlm_generic_payload rTrans;

  if (nbDma_Msk > line_DMA){
    sc_time delay = sc_time(0, SC_NS); //FIXME
    //fullfill payload
    rTrans.set_command(tlm::TLM_READ_COMMAND);
    rTrans.set_address(addr);
    rTrans.set_data_ptr(reinterpret_cast<unsigned char*>(buffer));
    rTrans.set_data_length(len);
    rTrans.set_dmi_allowed(false);
    rTrans.set_response_status( tlm::TLM_INCOMPLETE_RESPONSE );

#ifdef HWIP_DEBUG
    std::cout << "@ "<< sc_time_stamp() << " ==>"<< "HWIP send Read Transaction" << std::endl;
    std::cout << "\t CMD:"<< rTrans.get_command() << "\n";
    std::cout << "\t from addr: 0x"<<std::hex<< rTrans.get_address()<< "\n";
    std::cout << "\t to addr: 0x"<<std::hex<< (uintptr_t)rTrans.get_data_ptr()<< "\n";
    std::cout << "\t length: "<<std::dec<< rTrans.get_data_length() << std::endl;
#endif

    (*dma_Msk[line_DMA])->b_transport( rTrans, delay );
    assert(tlm::TLM_OK_RESPONSE == rTrans.get_response_status());
  }else{
    std::cerr << name() << "Request DMA line not available\n";
  }
}

void hwIP::memWrite(uint32_t addr, void* buffer, size_t len, int line_DMA)
{
  tlm::tlm_generic_payload wTrans;
  if (nbDma_Msk > line_DMA){
    sc_time delay = sc_time(0, SC_NS); //FIXME
    //fullfill payload
    wTrans.set_command(tlm::TLM_WRITE_COMMAND);
    wTrans.set_address(addr);
    wTrans.set_data_ptr(reinterpret_cast<unsigned char*>(buffer));
    wTrans.set_data_length(len);
    wTrans.set_dmi_allowed(false);
    wTrans.set_response_status( tlm::TLM_INCOMPLETE_RESPONSE );

#ifdef HWIP_DEBUG
    std::cout << "@ "<< sc_time_stamp() << " ==>"<< "HWIP send Write Transaction" << std::endl;
    std::cout << "\t CMD:"<< wTrans.get_command() << "\n";
    std::cout << "\t to addr: 0x"<<std::hex<< wTrans.get_address()<< "\n";
    std::cout << "\t from addr: 0x"<<std::hex<< (uintptr_t)wTrans.get_data_ptr()<< "\n";
    std::cout << "\t length:"<<std::dec<< wTrans.get_data_length() << std::endl;
#endif

    (*dma_Msk[line_DMA])->b_transport( wTrans, delay );
    assert(tlm::TLM_OK_RESPONSE == wTrans.get_response_status());
  }else{
    std::cerr << name() << "Request DMA line not available\n";
  }
}
