/* ---
 *  This file define an API for easily manage the NoC
 *
 *  Author <baptiste.roux AT inria.fr>
 * -------------------------------------------------------------------------- */
#include "nocAPI.h"

using namespace napi;
using namespace noc;


/* ---
 * nocAPI base class:
 *    Define base function to comunicate through HMpSoC NoC
 * ---                                                                       --- */

/* --- Constructor                                                           --- */
nocAPI::nocAPI(const std::string& device):
  deviceName(device){
//Open device driver
  noc_fd = open (deviceName.c_str(), O_RDWR);
  if (noc_fd < 1) {
    std::cerr << __PRETTY_FUNCTION__<<"Open "<<deviceName<<" failed\n";
    throw std::system_error();
  }
  // init random generator for task_id
  srand(time(NULL));

  //start nocDiscovery
  __discover();
}

nocAPI::~nocAPI(){
  close(noc_fd);
}

/* --- Primitives functions                                                  --- */
int8_t nocAPI::__discover(){
  print_DNAPI("called\n");
  uint32_t clProps[MAX_CLUSTER +1];
  int8_t status;

  // Ask value to driver
  status = ioctl(noc_fd, NOC_DISCOVER, clProps);

  // convert Raw data in map
  for( uint i=0; i<clProps[0]; i++){
    clPos cl_pos;
    cl_pos.x_pos = ((clProps[1+(2*i)]&0xFF00)>>8);
    cl_pos.y_pos = (clProps[1+(2*i)]&0x00FF);
    nocDiscover[clProps[(2*i)+2]] = cl_pos;
    if (0x0000 != (clProps[(2*i)+2]&0xffff)){
      nocLocalAddr = clProps[(2*i)+2]&0xffff0000;
    }
  }
#ifdef NAPI_DEBUG
  std::cout << __PRETTY_FUNCTION__ <<": <= local@0x"<<std::hex<<nocLocalAddr<<"\n";
  for( auto & it_NoC: nocDiscover){
    std::cout << "\t @0x" <<std::hex<< it_NoC.first
              <<std::dec<< " =>["<<(int)it_NoC.second.x_pos
              << ","<<(int)it_NoC.second.y_pos << "]\n";
  }
#endif
  return status;
}

int8_t nocAPI::__register(task_t task_id){
  print_DNAPI("called(0x"<<std::hex<<task_id<<")\n");
  int8_t status;
    status = ioctl(noc_fd, NOC_REGISTER, &task_id);
    if(status < 0){
      std::cerr << __PRETTY_FUNCTION__<<"Register task [0x"
                <<std::hex<<task_id<<"] failed.\n";
    }
  return status;
}

int8_t nocAPI::__unRegister(task_t task_id){
  print_DNAPI("called(0x"<<std::hex<<task_id<<")\n");
  int8_t status;
    status = ioctl(noc_fd, NOC_UNREGISTER, &task_id);
    if(status < 0){
      std::cerr << __PRETTY_FUNCTION__<<"UnRegister task [0x"
                <<std::hex<<task_id<<"] failed.\n";
    }
  return status;
}

int8_t nocAPI::__sendData(orderHeader order){
  print_DNAPI("called (task_id 0x"<<std::hex<< order.task_id<<", from_id 0x"<<order.sender_id
              <<", cmd 0x"<<order.cmd
              <<", local@0x"<<order.local<<", remote@0x"<<order.remote<<")\n");
  int8_t status;
  // generate packet_id and reset sync_header flags
  order.is_sync_header= false;
  status = ioctl(noc_fd, NOC_SEND, &order);
  return status;
}

orderHeader nocAPI::__waitOrder(waitOrder wo){
  print_DNAPI("called(trgt: 0x"<<std::hex<<wo.task_id<<", from: 0x"<<wo.from_id
              <<", Cmd 0x"<<wo.order_type<<")\n");
  int8_t status;
  orderHeader rcvBuffer;
  wo.data = &rcvBuffer;

  status = ioctl(noc_fd, NOC_WAITORDER, &wo);
  if(status < 0){
    std::cerr << __PRETTY_FUNCTION__<<" failed.\n";
    rcvBuffer.cmd = NOC_FAILED;
  }
  return(rcvBuffer);
}

ack_t nocAPI::__waitAck(waitPoint wp){
  print_DNAPI("called(0x"<<std::hex<<wp.task_id<<", Ack 0x"<<(ushort)wp.ack_type<<")\n");
  int8_t status;

  status = ioctl(noc_fd, NOC_WAITACK, &wp);
  if(status < 0){
    std::cerr << __PRETTY_FUNCTION__<<" failed.\n";
  }
  return (wp.ack_type);
}

int8_t nocAPI::__getPowerLog(float& eng, float& time){
  print_DNAPI("called\n");
  int8_t status;
  float buffer[2];

  status = ioctl(noc_fd, NOC_GETPOWER, buffer);
  if(status < 0){
    std::cerr << __PRETTY_FUNCTION__<<" failed.\n";
  }
  time = buffer[0];
  eng = buffer[1];
  return (status);
}

/* --- Utility functions                                                   --- */
uint nocAPI::getOffset(trgtId_t trgt){
  print_DNAPI("called(0x"<<std::hex<<trgt.first<<", @0x"<<trgt.second<<")\n");
  auto it = nocDiscover.lower_bound(trgt.second); //first element => key < it
  if((it->first) != trgt.second){ // it> key => need previous entry
    if(nocDiscover.begin() == it){
      std::cerr << "No cluster entry found in NoC" << std::endl;
      return 0;
    }else{it--;}
  }else{ /*it= key => right entry*/}

  //return offset
  return(trgt.second - it->first);
}

void nocAPI::sendAck(trgtId_t trgt, ack_t ack_type){
  print_DNAPI("("<<trgt.first <<", "<< trgt.second <<", "<<(ushort)ack_type<<")\n");
  orderHeader order;
  order.cmd = NOC_ACK;
  order.task_id = trgt.first;
  order.sender_id = 0x00; // not used
  order.remote = trgt.second;
  order.local = ack_type;

  // Send on nocItf
  __sendData(order);
  return;
}

void nocAPI::memGet(trgtId_t trgt, uintptr_t lpAddr, size_t len){
  print_DNAPI("("<<trgt.first <<", "<< trgt.second <<"," << lpAddr<<", "<< len <<")\n");
  orderHeader order;
  order.cmd = NOC_MEMRTV;
  order.task_id = trgt.first;
  order.sender_id = 0x00; //not used
  order.remote = trgt.second;
  order.local = nocLocalAddr + lpAddr;
  order.data_length = len;

  // Send on nocItf
  __sendData(order);
  return;
}

/*--- send & rcv data functions                                           --- */
task_t nocAPI::sendData(trgtId_t trgt, task_t from, void* data, size_t data_length){
  int8_t status;
  //NOTE: localAddr should be expressed in nocAddr domain =>(nocLocalAddr + offset in monMemory)
  //check Addr validity
  if (nocDiscover.end() == nocDiscover.find(trgt.second)) {
    std::cerr <<__PRETTY_FUNCTION__ << "Invalid trgtr cluster Address(@0x"
              << std::hex<< trgt.second <<"\n";
    return (-1);
  }

  // Setup order info
  uintptr_t pAddr_local = nocLocalAddr + monitored_VtoP(data);
  orderHeader order;
  order.cmd = NOC_MEMGET;
  uint pkt_tag = random()&NOC_PCKTMASK;
  order.task_id = (trgt.first&NOC_TASKMASK) + pkt_tag;
  order.sender_id = (from&NOC_TASKMASK) + pkt_tag;
  order.remote = trgt.second;
  order.local = pAddr_local;
  order.data_length = data_length;

  // Send on nocItf
  status = __sendData(order);
  if (status<0){
    std::cerr <<__PRETTY_FUNCTION__ << "Order write failed.\n";
    return (status& 0xffff); //error detect at n+1 if task_id MSB != jobId
  }else{
    if (MGMT_TASKID == (order.sender_id & NOC_TASKMASK)){ //Master MGMT register with trgt task_id
      return(order.task_id);
    }else{
      return(order.sender_id);
    }
  }
}

int8_t nocAPI::b_sendData(trgtId_t trgt, task_t from, void* data, size_t data_length){
  task_t status;
  status = sendData(trgt, from, data, data_length);
  if(((status&NOC_TASKMASK) != trgt.first) && (status&NOC_TASKMASK) != from){
    std::cerr <<__PRETTY_FUNCTION__ << "Send data failed.\n";
    return (-1);
  }
  print_DNAPI("Send task_id: 0x"<<std::hex<<status<<"\n");

  waitPoint wp;
  wp.task_id = status;
  wp.ack_type = NACK|PRC_ACK;

  ack_t answer = __waitAck(wp);
  if (PRC_ACK == answer)
    return(1);
  else if(NACK == answer)
    return(-1);
  else{
    std::cerr <<__PRETTY_FUNCTION__ << "Error: invalid ACK ("
              <<ack_helper.find(answer)->second << ")\n";
    return(-1);
  }
}

size_t nocAPI::b_rcvData(task_t task_id, task_t from_id, void* buffer, size_t maxSize){
  //poll on rcv order until memget rcv from job jobId
  waitOrder wo;
  wo.task_id = task_id;
  wo.from_id = from_id;
  wo.order_type = NOC_MEMGET;
  orderHeader orderRcv = __waitOrder(wo);
  if (NOC_FAILED == orderRcv.cmd){ return 0;}

  // send RCV_ACK
  if (MGMT_TASKID == (orderRcv.sender_id & NOC_TASKMASK)){ //Master MGMT register with trgt task_id
    orderRcv.sender_id = orderRcv.task_id;
  }
  trgtId_t trgt = std::make_pair(orderRcv.sender_id, orderRcv.local);
  // uint offset = getOffset(trgt);
  sendAck(trgt, RCV_ACK);

  if(orderRcv.data_length > maxSize){
    // Reject request
    sendAck(trgt, NACK);
    orderRcv.data_length = 0;
  }else{
    memGet(trgt, monitored_VtoP(buffer), orderRcv.data_length);
    //send PRC_ACK
    sendAck(trgt, PRC_ACK);
  }

  return((size_t)orderRcv.data_length);
}

    /*--- Sync Barriers functions                                             --- */
int8_t nocAPI::ptp_syncMaster(trgtId_t trgt, task_t from){
  int8_t status;
  //check Addr validity
  if (nocDiscover.end() == nocDiscover.find(trgt.second)) {
    std::cerr <<__PRETTY_FUNCTION__ << "Invalid trgtr cluster Address \n";
    return (-1);
  }

  // Setup order info
  orderHeader order;
  order.cmd = NOC_JOBSYNC;
  uint pkt_tag = random() & NOC_PCKTMASK;
  order.task_id = (trgt.first&NOC_TASKMASK) + pkt_tag;
  order.sender_id = (from &NOC_TASKMASK) + pkt_tag;
  order.remote = trgt.second;
  order.local = 0x00;
  order.data_length = 0x00;

  // Send on nocItf
  status = __sendData(order);
  if (status<0){
    std::cerr <<__PRETTY_FUNCTION__ << "Order write failed.\n";
    return (-1);
  }else{
    waitPoint wp;
    wp.task_id = order.task_id;
    wp.ack_type = NACK|PRC_ACK;

    ack_t answer = __waitAck(wp);
    if (PRC_ACK == answer)
      return(1);
    else if(NACK == answer)
      return(-1);
    else{
      std::cerr <<__PRETTY_FUNCTION__ << "Error: invalid ACK ("
                <<ack_helper.find(answer)->second << ")\n";
      return(-1);
    }
  }
}

int8_t nocAPI::ptp_syncReply(trgtId_t trgt, task_t from){
  //poll on rcv order until JOBSYNC rcv from job jobId
  waitOrder wo;
  wo.task_id = trgt.first;
  wo.from_id = from;
  wo.order_type = NOC_JOBSYNC;
  orderHeader orderRcv = __waitOrder(wo);
  if (NOC_FAILED == orderRcv.cmd){ return -1;}

  // send PRC_ACK
  sendAck(trgt, PRC_ACK);
  return(0);
}


/* ---
 * nocMaster class:
 *    Define extra method to use HMpSoC NoC as a master/manager
 * ---                                                                       --- */
nocMaster::nocMaster(const std::string& device):
  nocAPI(device){
}

nocMaster::~nocMaster(){
  for(const auto &task: linkedTasks){
    unregisterTask(task.first);
  }
}

/*--- Registration and spawning functions                                 --- */
int8_t nocMaster::registerTask(trgtId_t trgt){
  int8_t status=0;
  status = __register(trgt.first);

  if(0<= status){
    linkedTasks[trgt.first] = trgt.second;
  }
  return status;
}

int8_t nocMaster::unregisterTask(task_t task_id){
  int8_t status;

  auto inMap= linkedTasks.find(task_id);
  if(inMap == linkedTasks.end()){
    std::cerr << __PRETTY_FUNCTION__
              <<"Current nocMaster instance have no task[0x"
              <<std::hex <<task_id<<"] registered\n";
    return(-1);
  }else{
    status = __unRegister(task_id);
    if(0<= status){
      linkedTasks.erase(inMap);
    }
  }
  return status;
}

/*--- Subscribe functions                                                 --- */
int8_t nocMaster::askSlaveSubscribe(trgtId_t &trgt){
  int8_t status=0;
    //check Addr validity
    if (nocDiscover.end() == nocDiscover.find(trgt.second)) {
      std::cerr <<__PRETTY_FUNCTION__ << "Invalid cluster Address \n";
      return (-1);
    }
    print_DNAPI("Ask to Clusters: 0x"<<std::hex<<trgt.second<<"\n");

    //register task in drv to be able to waitOnAck
    registerTask(trgt);

    // Setup order info
    orderHeader order;
    order.cmd = NOC_JOBASK;
    //jobAsk packt_id always composed of clAddr MSB
    order.task_id = trgt.first;
    order.sender_id = trgt.first;
    order.remote = trgt.second;
    order.local = nocLocalAddr;

    // Send on nocItf
    status = __sendData(order);
    if (status >= 0){
      waitPoint wp;
      wp.task_id = order.task_id;
      wp.ack_type = NACK|PRC_ACK;
      ack_t answer = __waitAck(wp);
      if(NACK == answer){
        unregisterTask(trgt.first);
        return(0);
      }else if (PRC_ACK == answer)
        return(1);
      else{
        std::cerr <<__PRETTY_FUNCTION__ << "Error: invalid ACK ("
                  <<ack_helper.find(answer)->second << ")\n";
        return(0);
      }
    }else{
      return status;
    }
}

std::vector<trgtId_t> nocMaster::brdSubscribe(uint reqWorker, uint8_t jobId){
  //Send askSlaveSubscribe to every clusters until reqWorker respond yes
  std::vector<trgtId_t> worker;
  trgtId_t trgt;
  int8_t status=0;
  for( auto & it_NoC: nocDiscover){
    if(0x0000 == (it_NoC.first & 0xFFFF)){ // not localCluster => send ask
      trgt.first = (jobId <<24);
      trgt.second = it_NoC.first;
      status = askSlaveSubscribe(trgt);
      if( 0 <= status){
        worker.push_back(trgt);
      }
    }
  if(reqWorker == worker.size())
      return(worker);
  }
  return (worker);
}

/*--- retrieve noc power  functions                                       --- */
int8_t nocMaster::get_powerNoc(float &eng, float &time){
  return( __getPowerLog(eng, time));
}

/*--- Sync Barriers functions                                             --- */
std::vector<task_t> nocMaster::barrier_syncMaster(std::vector<task_t> trgt, task_t from){
  std::vector<task_t> errorTrgt;

  for(auto target:trgt){
    auto trgtAddr = linkedTasks.find(target);
      if(trgtAddr == linkedTasks.end()){
        std::cerr <<__PRETTY_FUNCTION__ <<
        "Warning: invalid target(0x"<<target<<"\n";
        errorTrgt.push_back(target);
        trgt.erase(std::remove(trgt.begin(), trgt.end(), target), trgt.end());
      }else{
        if(0>ptp_syncMaster(std::make_pair(target, trgtAddr->second), from))
          errorTrgt.push_back(target);
      }
  }
  //all slave arrived at the meeting point: unlock them
  for(auto target:trgt){
    auto trgtAddr = linkedTasks.find(target);
      if(trgtAddr == linkedTasks.end()){
        std::cerr <<__PRETTY_FUNCTION__ <<
        "Warning: invalid target(0x"<<target<<"\n";
        errorTrgt.push_back(target);
      }else{
        if(0>ptp_syncMaster(std::make_pair(target, trgtAddr->second), from))
          errorTrgt.push_back(target);
      }
  }
  return errorTrgt;
}

/* ---
 * nocSlave class:
 *    Define extra method to use HMpSoC NoC as slave.
 *    Link to a given task at construction. Do a given job and return.
 * ---                                                                       --- */
nocSlave::nocSlave(trgtId_t mId, const std::string& device ):
  nocAPI(device),
  master_id(mId){
  __register(master_id.first);
  print_DNAPI("Slave id: 0x"<<std::hex<<master_id.first
            <<" generated by Cluster@0x"<< master_id.second<<"\n");
  }

nocSlave::~nocSlave(){
  __unRegister(master_id.first);
}

/*--- endPoint functions                                                   --- */
orderHeader nocSlave::ep_waitJob(){
  waitOrder wo;
  wo.task_id = master_id.first;
  wo.from_id = 0x00;
  wo.order_type = NOC_JOBASK;
  orderHeader orderRcv = __waitOrder(wo);
  if (NOC_FAILED != orderRcv.cmd){
    trgtId_t trgt = std::make_pair(orderRcv.sender_id, orderRcv.local);
    sendAck(trgt, RCV_ACK);
  }
  return orderRcv;
}

void nocSlave::job_answer(trgtId_t trgt, ack_t ack){
    sendAck(trgt, ack);
    return;
}

/*--- Sync Barriers functions                                             --- */
int8_t nocSlave::barrier_syncReply(trgtId_t trgt, task_t from){
  //slave barrier is a two phase lock with master:
  //    master wait all thread in the barrier then unlock them
  if(0>  ptp_syncReply(trgt, from)){return (-1);}
  if(0>  ptp_syncReply(trgt, from)){return (-1);}
  return 0;
}
