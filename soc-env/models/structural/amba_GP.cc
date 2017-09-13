#include "amba_GP.h"

/*---
 * METHODS IMPLEMENTATION
 *----------------------------------------------------------------------------*/
amba_GP::amba_GP(sc_core::sc_module_name name, uint8_t nbMsk, cereal::serialMap<sb::hwCpnCnf> hwCpn)
  : sc_module(name), cpu_Ssk("cpu_Ssk"), nb_Msk(nbMsk), monSysSlave(1)//only one monitored channel
{
  //register slave socket callback functions
  cpu_Ssk.register_b_transport(this, &amba_GP::b_transport);

  //Create Msk and addrToSk following HwCpn configuration
  pl_Msk = new tlm_utils::simple_initiator_socket<amba_GP>*[nbMsk];
  int trgt_socket=0;
  size_t max_size = 0;
  for(const auto & it_hw: hwCpn.sMap)
    {
      pl_Msk[trgt_socket] = new tlm_utils::simple_initiator_socket<amba_GP>(it_hw.first.c_str());
      addrToSk[it_hw.second.addr_offset] = trgt_socket++;
      max_size = ((it_hw.second.addr_offset+ it_hw.second.addr_space)>max_size)
        ?(it_hw.second.addr_offset+ it_hw.second.addr_space):max_size;
    }
  size = max_size;
  amba_mem = new uint8_t[size];

#ifdef AMBA_DEBUG
  std::cout << name<<" Created " << std::endl;
#endif
}

amba_GP::~amba_GP()
{
  delete[] amba_mem;

  for (int sk=0 ; sk< nb_Msk; sk++){
    delete pl_Msk[sk];
  }
  delete[] pl_Msk;
}

void amba_GP::b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
{
  /*
   * Append communication to monitor system and update message cost
   */
  uint tmp = comLog(trans.get_data_length(), 0);
  delay += sc_time(tmp,SC_NS);


  //update trans addr to IPHW @space and return socketId
  uint sk_out;
  sk_out = updtTransToSubspace(trans);

  //forward transaction to matching socket
  (*pl_Msk[sk_out])->b_transport(trans, delay);
}

uint amba_GP::updtTransToSubspace(tlm::tlm_generic_payload &trans){
  sc_dt::uint64 addr = trans.get_address();

#ifdef AMBA_DEBUG
  std::cout << name()<< ": \n";
  std::cout << " TLM request on 0x"<<std::hex<<addr <<"\n";
#endif
  if (addr > size){
    SC_REPORT_FATAL(name(), "Bad memory access no route find to target IP (address too high)\n");
  }else{
    std::map<sc_dt::uint64, uint>::const_iterator it;
    it = addrToSk.lower_bound(addr); //first element that is not less than key.
    if((it->first) != addr){ // it> key => need previous entry
      if(it == addrToSk.begin()){
        SC_REPORT_FATAL(name(), "Bad memory access no route find to target IP (address too low)\n");
      }else{
        it--;
      }
    }else{ /*it= key => right entry*/}

#ifdef AMBA_DEBUG
    std::cout << "\t find entry on sk: 0x"<<std::hex<< it->second<< "\n";
    std::cout << "\t entry offset: 0x"<<std::hex<< it->first<< "\n";
#endif

    //update trans addr to msSlave @space and return socketId
    addr -= it->first;
    trans.set_address(addr);
    return(it->second);
  }
}
