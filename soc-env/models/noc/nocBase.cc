#include "nocBase.h"
using namespace noc;

nocItf_base::nocItf_base(sc_core::sc_module_name name, std::map<uint, noc::clPos>nocDiscover)
  : sc_module(name), cmd_Ssk("cmdSk"), DMA_Msk("dmaSk"), nocDiscover(nocDiscover)
{
  // register tlm callback
  cmd_Ssk.register_b_transport(this, &nocItf_base::b_transport);

  // allocate memory for register and init value
  //allocate register memory
  noc_register = new uint8_t[RNOC_POOLSIZE];
  memset(noc_register, 0x00, sizeof(uint8_t)*RNOC_POOLSIZE);
}

nocItf_base::~nocItf_base() {
  delete[] noc_register;
}

nocRouter_base::nocRouter_base(int nbSk, sb::nocCnf config)
  : nb_sk(nbSk), nocConfig(config)
{}


uint nocRouter_base::toClusterAddr(uint64_t &addr){
  uint64_t clAddrOffset;
  uint clSk;

  std::map<uint64_t, uint>::const_iterator it;
  it = clMapOnSk.lower_bound(addr); //first element => key < it
  if((it->first) != addr){ // it> key => need previous entry
    if(clMapOnSk.begin() == it){
      std::cerr << "Router: required addr(0x"<<std::hex<<addr<<") to low" << std::endl;
      exit(-1);
    }else{it--;}
  }else{ /*it= key => right entry*/}

#ifdef NOC_DEBUG_BIND
  std::cout << "Router: \n";
  std::cout << " Request on 0x"<<std::hex<<addr <<"\n";
  std::cout << "\t find entry on sk: 0x"<<std::hex<< it->second<< "\n";
  std::cout << "\t entry offset: 0x"<<std::hex<< it->first<< "\n";
#endif

  //update addr to SoC @space and return socketId
  addr -= it->first;
  return(it->second);
}

uint nocRouter_base::nHop(clPos from, clPos to){
  uint n_hop;
  switch(nocConfig.topo){
  case RING:{
    uint delta;
    delta = abs((from.x_pos + (nocConfig.xSize)*from.y_pos) -(to.x_pos + (nocConfig.xSize)*to.y_pos));
    n_hop = std::min(delta, (uint)abs((int)((nocConfig.xSize*nocConfig.ySize)-delta)));
    break;
  }
  case MESH_2D:{
    n_hop = abs(from.x_pos-to.x_pos) + abs(from.y_pos-to.y_pos);
    break;
  }
  case TORUS_2D:{
    uint x_hop, y_hop;
    x_hop = std::min(abs(from.x_pos -to.x_pos), nocConfig.xSize -abs(from.x_pos-to.x_pos));
    y_hop = std::min(abs(from.y_pos -to.y_pos), nocConfig.ySize -abs(from.y_pos-to.y_pos));
    n_hop = x_hop + y_hop;
    break;
  }
  }
  return n_hop;
}

void nocRouter_base::comLog(uint fromSk, uint toSk , size_t size){
  clPos from, to;
  uint hops, fullPacket, residual;
  bool existResidual;
  uint tnsCost;
  float enjCost;

  //Get cluster position
  auto itFrom = skToClpos.find(fromSk);
  auto itTo = skToClpos.find(toSk);
  ASSERT(skToClpos.end() != itFrom,"socket"<<fromSk<<"have no matching clPos");
  ASSERT(skToClpos.end() != itTo,"socket"<<toSk<<"have no matching clPos");
  from = itFrom->second;
  to = itTo->second;

  //get hop in network
  hops = nHop(from, to);
  //Split in packet
  fullPacket = (size/nocConfig.params.maxPacketLength);
  residual = (size % nocConfig.params.maxPacketLength);
  existResidual = (0==residual)?false:true;
  //compute communication cost
  tnsCost = (fullPacket *(nocConfig.params.tns_atm
                         + hops*(nocConfig.params.tns_rcc + nocConfig.params.tns_sbc*nocConfig.params.maxPacketLength))
             +existResidual*(nocConfig.params.tns_atm + hops*(nocConfig.params.tns_rcc + nocConfig.params.tns_sbc*residual)));
  enjCost = (fullPacket *(nocConfig.params.enj_atm
                         + hops*(nocConfig.params.enj_rcc + nocConfig.params.enj_sbc*nocConfig.params.maxPacketLength))
             +existResidual*(nocConfig.params.enj_atm + hops*(nocConfig.params.enj_rcc + nocConfig.params.enj_sbc*residual)));

  //append cost on counter
  enj_log += enjCost;
  tns_log += tnsCost;

#ifdef NOC_DEBUG_LOG
  std::cout << "NoC communication occur:\n\tpower"<< enjCost<<"\n";
  std::cout << "\ttime"<< tnsCost<<"\n";
#endif

  return;
}
