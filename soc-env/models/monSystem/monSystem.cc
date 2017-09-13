#include "monSystem.h"

/*---
 * CLASS MONSYSSLAVE
 * METHODS IMPLEMENTATION
 *----------------------------------------------------------------------------*/
using namespace ms;
using namespace std;

monSysSlave::monSysSlave(uint nbChan)
  : ms_Ssk("msSlave_Socket"),
    nb_chan(nbChan),engCounter(0.0), timeCounter(0.0)
{
  //register slave socket callback functions
  ms_Ssk.register_b_transport(this, &monSysSlave::ms_b_transport);

  //allocate params struct for each channel
  channels = new params[nbChan];
}

monSysSlave::~monSysSlave()
{
  delete[] channels;
}

void monSysSlave::ms_b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
{
#ifdef MS_DEBUG
  std::cout << "@ "<< sc_time_stamp() << " ==>"<< "Slave monitoring system received transaction" << std::endl;
  std::cout << "\t addr: 0x"<< hex<< trans.get_address()<< std::endl;
  std::cout << "\t CMD: 0x"<< hex<< trans.get_command() << std::endl;
  std::cout << "\t length: 0x"<< hex<< trans.get_data_length() << std::endl;
#endif

  tlm::tlm_command cmd = trans.get_command();
  sc_dt::uint64    addr = trans.get_address();
  unsigned char*   ptr = trans.get_data_ptr();
  unsigned int     len = trans.get_data_length();
  unsigned int real_len;

  if (cmd == tlm::TLM_READ_COMMAND){
    if (sc_dt::uint64(0x00) == addr){ //energy counter
      memcpy(ptr, &engCounter, sizeof(float));
#ifdef MS_DEBUG
      std::cout << "Send energy counter: "<<dec << engCounter<<" \n";
#endif
    }else if (sc_dt::uint64(0x04) == addr){ //time counter
      memcpy(ptr, &timeCounter, sizeof(float));
#ifdef MS_DEBUG
      std::cout << "Send time counter: "<<dec << timeCounter<<" \n";
#endif
    } else if(addr <= sc_dt::uint64(SLAVE_MEM_SIZE(nb_chan))){
      //Sub eng counter and time counter to trgt addr
      addr -= 2*sizeof(float);
      // burst only on one channel parameters
      real_len = (len>(sizeof(params) - addr%sizeof(params)))
        ?(sizeof(params) -addr%sizeof(params)):len;
      memcpy(ptr, (void*)((uintptr_t)(channels)+addr), real_len);
#ifdef MS_DEBUG
      std::cout << "Send params values\n";
#endif
    }else{
#ifdef MS_DEBUG
      std::cout << "Access on @: 0x"<< hex<< addr << "; Max request is: 0x"<<SLAVE_MEM_SIZE(nb_chan)<<"\n";
#endif
      trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
      SC_REPORT_FATAL("monSysSlave", "Unsupported read access\n");
      return;
    }
  }
  else if (cmd == tlm::TLM_WRITE_COMMAND){
    //reset counter on register write
    engCounter = 0.0;
    timeCounter = 0.0;
    if( addr < 2*sizeof(float)){
      //can't write into counter register => just reset it
    }
    else if(addr <= sc_dt::uint64(SLAVE_MEM_SIZE(nb_chan))){
        //Sub eng counter and time counter to trgt addr
        addr -= 2*sizeof(float);
        // burst only on one channel parameters
        real_len = (len>(sizeof(params) - addr%sizeof(params)))
          ?(sizeof(params) -addr%sizeof(params)):len;
        memcpy((void*)((uintptr_t)(channels)+addr), ptr, real_len);
      }else{
      trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
      SC_REPORT_FATAL("monSysSlave", "Unsupported write access\n");
      return;
    }
  }
  trans.set_dmi_allowed(false);
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

uint monSysSlave::comLog(size_t comSize, uint comChan)
{
  //assert if enough channels have been set in configuration file
  assert(comChan <= nb_chan);

  uint fullBurst;
  uint residual;
  uint existResidual = 0;
  uint time_ns;

  //Split in burst packet
  fullBurst = (comSize/channels[comChan].maxBurst);
  residual = (comSize % channels[comChan].maxBurst);
  existResidual = (0==residual)?false:true;

  //compute communication time
  time_ns = (fullBurst *(channels[comChan].timeS_ns + channels[comChan].maxBurst*channels[comChan].timeD_ns)
             +existResidual*( channels[comChan].timeS_ns + residual*channels[comChan].timeD_ns));

  // Inc counter
  timeCounter += (float)(NS_TO_S *time_ns);
  engCounter += (float) (fullBurst *(channels[comChan].engS + channels[comChan].maxBurst*channels[comChan].engD)
                         +existResidual*( channels[comChan].engS + residual*channels[comChan].engD));

#ifdef MS_DEBUG
  std::cout << "callback on comLog: \n"
    << "\t request of size 0x"<<std::hex<<comSize
    <<std::dec<<" => ["<<engCounter<< ", " << timeCounter<< "]\n";
#endif

  return time_ns;
}

/*---
 * CLASS MONSYSMASTER
 * METHODS IMPLEMENTATION
 *----------------------------------------------------------------------------*/
monSysMaster::monSysMaster(sc_core::sc_module_name name, uint8_t nbMsk, ms::clusterProp clProps)
  : sc_module(name), nb_Msk(nbMsk), high_addr(0), cl_Props(clProps)
  ,cpu_Ssk("cpu_SSk")
{
  //Cpu slave socket
  cpu_Ssk.register_b_transport(this, &monSysMaster::ms_b_transport);

  //Ip master Sockets
  ip_Msk = new tlm_utils::simple_initiator_socket<monSysMaster>*[nb_Msk];
  uint trgt_socket=0;
  ulong chan_offset=GLOBAL_REGSIZE; //reserved space for global values
  ip_Msk[trgt_socket] = new tlm_utils::simple_initiator_socket<monSysMaster>("ram");
  addrToSk[chan_offset]=trgt_socket;
  cl_Props.memory.blk_addr = chan_offset;
  chan_offset += (SLAVE_MEM_SIZE(cl_Props.memory.comChan.sMap.size()));
  ++trgt_socket;

  ip_Msk[trgt_socket] = new tlm_utils::simple_initiator_socket<monSysMaster>("ambaGP");
  addrToSk[chan_offset]=trgt_socket;
  cl_Props.amba.blk_addr = chan_offset;
  chan_offset += (SLAVE_MEM_SIZE(cl_Props.amba.comChan.sMap.size()));
  ++trgt_socket;

  for(auto & it_mBlks: cl_Props.hwCpn.sMap)
    {
      ip_Msk[trgt_socket] = new tlm_utils::simple_initiator_socket<monSysMaster>(it_mBlks.first.c_str());
      addrToSk[chan_offset]=trgt_socket;
      it_mBlks.second.blk_addr = chan_offset;
      chan_offset += (SLAVE_MEM_SIZE(it_mBlks.second.comChan.sMap.size()));
      ++trgt_socket;
    }
  high_addr=chan_offset;
}

monSysMaster::~monSysMaster()
{
  for (uint sk=0 ; sk< nb_Msk; sk++){
    delete ip_Msk[sk];
  }
  delete[] ip_Msk;
}

void monSysMaster::ms_b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
{
#ifdef MS_DEBUG
  std::cout << name()<< ": @"<< sc_time_stamp() <<"\n";
  std::cout << "TLM transaction received @:0x"<<std::hex<< trans.get_address()<< "\n";
#endif

  if (trans.get_address() < GLOBAL_REGSIZE){ //request on global
    float globalVal = rtvGlobalLog(trans);

    // update value and specify end of transaction
    unsigned char*   ptr = trans.get_data_ptr();
    memcpy(ptr, &globalVal, sizeof(float));
    trans.set_response_status(tlm::TLM_OK_RESPONSE);

  } else { //foward to specific slave
  //forward transaction on right socket
  uint sk_out;
  sk_out = updtTransToSubspace(trans);
  (*ip_Msk[sk_out])->b_transport(trans, delay);
#ifdef MS_DEBUG
  std::cout << "\t => forward on sk:"<<dec<<sk_out<<" @:0x"<<std::hex<< trans.get_address()<< std::endl;
#endif
  }
}

void monSysMaster::sendParamsTrans(uint blkAddr, uint chanNum, params *blkParams){
  tlm::tlm_generic_payload trans;
  sc_time delay = sc_time(0, SC_NS);

  //setup TLM transaction
  trans.set_command(tlm::TLM_WRITE_COMMAND);
  trans.set_address((sc_dt::uint64) SLAVE_MEM_SIZE(chanNum));
  trans.set_data_ptr(reinterpret_cast<unsigned char*>(blkParams));
  trans.set_data_length(sizeof(params));
  trans.set_dmi_allowed(false);
  trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

  // send it on right socket
  (*ip_Msk[addrToSk[blkAddr]])->b_transport(trans, delay);
}

float monSysMaster::rtvGlobalLog(tlm::tlm_generic_payload &trans){
  //save initial data pointer
  unsigned char * dataPtr = trans.get_data_ptr();
  sc_time delay = sc_time(0, SC_NS);

  //retrieve eng or time from each slave or reset them
  float tmp = 0.0;
  float accumulator = 0.0;
  trans.set_data_ptr((unsigned char *) &tmp);
  for (auto slave: addrToSk){
    (*ip_Msk[slave.second])->b_transport(trans, delay);
    accumulator += tmp;
  }
  //reset initial data pointer value
  trans.set_data_ptr(dataPtr);
  return accumulator;
}

uint monSysMaster::updtTransToSubspace(tlm::tlm_generic_payload &trans){
  sc_dt::uint64 addr = trans.get_address();
  std::map<sc_dt::uint64, uint>::const_iterator it;
  it = addrToSk.lower_bound(addr); //first element that is not less than key.
  if((it->first) != addr){ // it> key => need previous entry
   it--;
  }else{ /*it= key => right entry*/}

#ifdef MS_DEBUG
  std::cout << name()<< "\n";
  std::cout << " TLM request on 0x"<<std::hex<<addr <<"\n";
  std::cout << "\t find entry on sk: 0x"<<std::hex<< it->second<< "\n";
  std::cout << "\t entry offset: 0x"<<std::hex<< it->first<< "\n";
#endif

  //update trans addr to msSlave @space and return socketId
  addr -= it->first;
  trans.set_address(addr);
  return(it->second);
}

void monSysMaster::monSysInit()
{
  { int chanNum=0;
    ulong blkAddr = cl_Props.amba.blk_addr;
    for(auto chan: cl_Props.amba.comChan.sMap){
      sendParamsTrans(blkAddr, chanNum++, &(chan.second));
    }
  }//init amba

  { int chanNum=0;
    uint blkAddr = cl_Props.memory.blk_addr;
    for(auto chan: cl_Props.memory.comChan.sMap){
      sendParamsTrans(blkAddr, chanNum++, &(chan.second));
    }
  }//init memory

  for(auto & it_hw: cl_Props.hwCpn.sMap){
    int chanNum=0;
    uint blkAddr = it_hw.second.blk_addr;
    for(auto chan: it_hw.second.comChan.sMap){
      sendParamsTrans(blkAddr, chanNum++, &(chan.second));
    }
  }
}

ulong monSysMaster::getAddrSpace(){
#ifdef MS_DEBUG
  std::cout << name()<< ": list of @space on sockets" <<"\n";
  for(const auto & it: addrToSk){
    std::cout <<hex<<"\t addrToSk[0x" << it.first << "]: " << dec<< it.second << "\n";
  }
  std::cout << std::endl;
#endif
  return ((ulong)high_addr);
}


void monSysMaster::exportClProp(std::ofstream &stream, char sType){
  if (stream.is_open()){
    switch (sType){
    case 'j' : //"JSON"
      {
        cereal::JSONOutputArchive oarchive(stream);
        cl_Props.save(oarchive);
        break;
      }
    case 'x' : //"XML"
      {
        cereal::XMLOutputArchive oarchive(stream);
        cl_Props.save(oarchive);
        break;
      }
    default:
      SC_REPORT_FATAL("monSysMaster", "Unsupported export type (options are j: JSON, x: XML)\n");
      break;
    }
  }else{
    SC_REPORT_FATAL("monSysMaster", " Output stream not open correctly \n");
  }
  return;
}
