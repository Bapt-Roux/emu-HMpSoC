#include "nocIpc.h"
using namespace noc;

/*---
 * Error management wrapper function for send and recv over TCP socket
 * ------------------------------------------------------------------------- */
uint ipc::erFree_recv(int socket, char* trgt, size_t size){
  char buffer[size];
  uint offset=0;
  int status=0;
  while(offset < size){
    status = recv(socket, &buffer[offset],size, 0x00/*NO_FLAGS*/);
    if(status>=0){
      offset+=status;
    }
#ifdef NIPC_DEBUG_IPC
    else{
      perror("Recv received signal:\n");
    }
#endif
  }
  //output read data on dest pointer
  memcpy(trgt, buffer,size);
  return offset;
}

uint ipc::erFree_send(int socket, char* src, size_t size){
  uint offset=0;
  int status=0;
  while(offset < size){
    status = send(socket, &src[offset],size, MSG_DONTWAIT);
    if(status>=0){
      offset+=status;
    }
#ifdef NIPC_DEBUG_IPC
    else{
      perror("Send received signal:\n");
    }
#endif
  }
  return offset;
}

/*---
 * NOC Interfaces implementation
 * ------------------------------------------------------------------------- */
ipc::nocItf::nocItf(sc_core::sc_module_name name, std::map<uint, noc::clPos>nocDiscover, uint preboot_ms, uint postboot_us)
  :nocItf_base(name, nocDiscover), trgtAddr(NULL), preBoot_ms(preboot_ms), postBoot_us(postboot_us),
   irqOrder_pending(0), irqWake_pending(0)
{
  //Alloate memory for callBack payload buffer
  cb_payload = new char[MAX_CB_PAYLOAD_LEN];
  SC_THREAD(syncUnixMsg);
  sensitive << syncEvt;

  SC_THREAD(order_irqRcvEvent);
  sensitive << orderEvt_rcv;

  SC_THREAD(wake_irqRcvEvent);
  sensitive << wakeEvt_rcv;
}

ipc::nocItf::~nocItf() {
  closeUnixIpc();
  delete[] cb_payload;
}

void ipc::nocItf::order_irqRcvEvent(){
  while(1){
    wait();
    irqOrder_pending.post();
    *((uint32_t*)_RNOC(RCVORDER))= irqOrder_pending.get_value(); // updt register
    irq_orderRcv.write(1);
    if (rcvOrder.size()> irqOrder_pending.get_value()){
      orderEvt_rcv.notify(rcvOrder[irqOrder_pending.get_value()].deltaInCycle_ns, SC_NS);
    }//else wait nxt notify from noc sync period
  }
}

void ipc::nocItf::wake_irqRcvEvent(){
  while(1){
    wait();
    irqWake_pending.post();
    *((uint32_t*)_RNOC(WAKEUP))= irqWake_pending.get_value(); // updt register
    irq_wakeUp.write(1);
    if (wakeupTask.size()> irqWake_pending.get_value()){
      wakeEvt_rcv.notify(wakeupDeltaInCycle[irqWake_pending.get_value()], SC_NS);
    }//else wait nxt notify from noc sync period
  }
}

void ipc::nocItf::openUnixIpc(std::string socketPath){
  if( NULL != trgtAddr){
    std::cerr << "IPC socket already open on this nocItf \n";
    return;
  }else{
    trgtAddr = new sockaddrUnix;
    trgtAddr->sun_family = AF_UNIX;
    strcpy(trgtAddr->sun_path, socketPath.c_str());

    int trgtDscLen= socketPath.length()+sizeof(trgtAddr->sun_family);

    if ((sockfd = socket(AF_UNIX, SOCK_STREAM,0)) < 0){
      perror("Creating socket");
      exit(0);
    }
    if (connect(sockfd, (struct sockaddr *)
                trgtAddr, trgtDscLen) < 0){
      perror("Connecting");
      exit(0);
    }
  }
}

void ipc::nocItf::closeUnixIpc(){
  if (NULL != trgtAddr){
    close(sockfd);
    delete trgtAddr;
  }
}

void ipc::nocItf::syncUnixMsg(){
  int status;
  tlm::tlm_generic_payload trans;
  orderHeader order;
  dcbHeader dcb;

  //Noc synchronisation start after a bunch of time to enhance linux boot time
  syncEvt.notify(preBoot_ms, SC_MS);
  wait();
  std::cout << sc_time_stamp() << " => " << name() <<" Enter postBoot phase"<<std::endl;

  while(1){
    syncEvt.notify(postBoot_us, SC_US);
#ifdef NIPC_DEBUG_IPC
    std::cout << sc_time_stamp() << " => " << name() <<" start sync sequence"<<std::endl;
#endif
    /*
     * End of send_order phase: TCP stream have been fullfil by tlm_transport callback function
     * Start handshake with syn header
     */
    order.is_sync_header= true;
    status = erFree_send(sockfd, reinterpret_cast<char*>(&order), sizeof(orderHeader));
    ASSERT(sizeof(orderHeader) == status,"get "<<dec << status <<" expected "<<sizeof(orderHeader));

    /*
     * Rcv order phase: read TCP Stream until SyncOrder header
     */
    double prv_deltaOrder_ns = 0.0;
    double prv_deltaWake_ns = 0.0;
    while(1){
      status = erFree_recv(sockfd, reinterpret_cast<char*>(&order), sizeof(orderHeader));
      if(!(order.is_sync_header)){
#ifdef NIPC_DEBUG_SYNC
        std::cout << sc_time_stamp() << "\t => " << name() << ": order header received \n";
        std::cout<< name()<<"cmd: "<< order.cmd<<"Delta in cycle value =>"<<order.deltaInCycle_ns<<"\n";
#endif
        //Process Order => generate dataCallback transaction if needed
        switch(order.cmd){
        case NOC_MEMRTV: { //generate callback
#ifdef NIPC_DEBUG_SYNC
          std::cout<< "NOC_MEMRTV Order \n";
#endif
          //get data with TLM transaction
          trans.set_address(order.remote);
          trans.set_command(tlm::TLM_READ_COMMAND);
          trans.set_data_length(order.data_length);
          trans.set_data_ptr(reinterpret_cast<unsigned char*>(cb_payload));
          sc_core::sc_time time(0, SC_NS); //issue request with 0 delay to prevent breaking ipc pipe => delay report at end of callback in receiver cluster
          DMA_Msk->b_transport(trans,time);
          //send it on router within a callback packet
          dcb.is_sync_header = false;
          dcb.remote = order.local;
          dcb.data_length = order.data_length;
          dcb.deltaInCycle_ns = order.deltaInCycle_ns;
          status = erFree_send(sockfd, reinterpret_cast<char*>((dcbHeader *)&dcb), sizeof(dcbHeader));
          ASSERT(sizeof(dcbHeader) == status,"get "<<dec << status <<" expected "<<sizeof(dcbHeader));
          status = erFree_send(sockfd, reinterpret_cast<char*>(cb_payload), dcb.data_length);
          ASSERT(dcb.data_length == status,"get "<<dec << status <<" expected "<<dcb.data_length);
          break;
        }
        case NOC_ACK: {
#ifdef NIPC_DEBUG_SYNC
          std::cout<< name()<<"NOC_ACK Order [0x"<<std::hex<<order.task_id<<"]("
                   <<  ack_helper.find(order.local)->second<< ") \n";
          std::cout << "\t" << "Pending Order list: \n";
          for (const auto & it_order: pendingOrder){
            std::cout << "\t \t" <<it_order.first <<"=> " << it_order.second <<" \n";
          }
          std::cout << "\t" << "Pending Wait list: \n";
          for (const auto & it_wait: pendingWait){
          std::cout << "\t \t" <<it_wait.first <<"=> " << it_wait.second <<" \n";
        }
          std::cout << "\t" << "WakeUp list: \n";
          for (const auto & it_wakeUp: wakeupTask){
            std::cout <<std::hex << "\t \t0x"<<it_wakeUp.task_id
                      <<"("<< ack_helper.find(it_wakeUp.ack_type)->second << ") \n";
          }
#endif
          // apply ack on order list
          auto itOrder = pendingOrder.find(order.task_id);
          if(pendingOrder.end() != itOrder) { //key exist
            // if((NACK == order.local)||(PRC_ACK == order.local)) //NOTE: add mask to trgt only available flag ?
            if(PRC_ACK == order.local) //Clear on PRC but keep NACK
              pendingOrder.erase(itOrder); //delete entry
            else
              itOrder->second &= ~order.local;
          }else{
#ifdef NIPC_DEBUG_SYNC
            std::cerr << name()<< ": Error received an ack: "<< ack_helper.find(order.local)->second
                      <<" for unknow task [task_id 0x"<<std::hex<<order.task_id
                      <<" ;sender_id 0x"<< order.sender_id<<"], or already process \n";
#endif
          }
          // Check if a process waiting on this event
          auto itWait = pendingWait.find(order.task_id);
          if(pendingWait.end() != itWait) { //key exist
            if((order.local & itWait->second)>0){ //wait pending on this flag
              //Add task in wakeupTask and generate irq
              waitPoint waitReturn;
              waitReturn.task_id = order.task_id;
              waitReturn.ack_type = (ack_t) order.local;
              wakeupTask.push_back(waitReturn);
              //update current delta_cycle as offset between packet => Wake delta linked dependency
              wakeupDeltaInCycle.push_back(order.deltaInCycle_ns - prv_deltaWake_ns);
              prv_deltaWake_ns = order.deltaInCycle_ns;
              wakeEvt_rcv.notify(order.deltaInCycle_ns, SC_NS);

              if (RCV_ACK == order.local) //reset flag
                itWait->second &= ~order.local;
              else //clear entry
                pendingWait.erase(itWait);
            }
          }
          //else no task to wakeup
          break;
        }
        case NOC_JOBASK:
        case NOC_JOBSYNC:
        case NOC_MEMGET: {
#ifdef NIPC_DEBUG_SYNC
          std::cout<< "NOC_JOBASK || NOC_JOBSYNC ||NOC_MEMGET Order \n";
#endif
          //update current delta_cycle as offset between packet => order delta linked dependency
          double abs_deltaOrder_ns = order.deltaInCycle_ns;
          order.deltaInCycle_ns -= prv_deltaOrder_ns;
          prv_deltaOrder_ns = abs_deltaOrder_ns;
          rcvOrder.push_back(order);
          orderEvt_rcv.notify(abs_deltaOrder_ns, SC_NS);
          break;
        }
        }
      } else{ /* syncRcv occur*/
        //update noc powerLog
        enj_noc = *(reinterpret_cast<float*>(&order.remote)); //extract float from uint64_t transport type
        tns_noc = order.local;
        //Append sync dataCallback on TCP stream
        dcb.is_sync_header= true;
        status = erFree_send(sockfd, reinterpret_cast<char*>(&dcb), sizeof(dcbHeader));
        ASSERT(sizeof(dcbHeader) == status,"get "<<dec << status <<" expected "<<sizeof(dcbHeader));
        break;
      }
    }

    /*
     * Rcv callback phase: read TCP Stream until SyncCallback header
     */
    while(1){
      dcb.is_sync_header=true;
      status = erFree_recv(sockfd, reinterpret_cast<char*>(&dcb), sizeof(dcbHeader));
      ASSERT(sizeof(dcbHeader) == status,"get "<<dec << status <<" expected "<<sizeof(dcbHeader));
      if(!(dcb.is_sync_header)){
#ifdef NIPC_DEBUG_SYNC
        std::cout << sc_time_stamp() << "\t => " << name() << ": callback header received \n";
        std::cout <<"\t remote: 0x" << hex << dcb.remote <<"\n";
        std::cout <<"\t data_length: 0x" << hex<< dcb.data_length <<"\n";
#endif
        //Get payload
        status = erFree_recv(sockfd, reinterpret_cast<char*>(cb_payload), dcb.data_length);
        ASSERT(dcb.data_length == status,"get "<<dec << status <<" expected "<<dcb.data_length);
        //Convert callback in TLM transaction
        trans.set_address(dcb.remote);
        trans.set_command(tlm::TLM_WRITE_COMMAND);
        trans.set_data_length(dcb.data_length);
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(cb_payload));
        sc_core::sc_time time(dcb.deltaInCycle_ns, SC_NS);
        DMA_Msk->b_transport(trans,time);
      } else{ /* syncRcv occur*/
        break;
      }
    }
    wait(); // wait next sync period
  }
}
void ipc::nocItf::b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
{
  task_t task;
  orderHeader order;
  waitPoint waitDesc;
  int status;
#ifdef NIPC_DEBUG_TRANS
  std::cout << "@ "<< sc_time_stamp() << " => " << name() << " TLM Transaction received \n";
  std::cout << "Transaction properties:\n";
  std::cout << "\t addr: 0x"<<hex<< trans.get_address()<<"\n";
  std::cout << "\t CMD: 0x"<< hex<< trans.get_command() <<"\n";
  std::cout << "\t length:"<<dec << trans.get_data_length() <<"\n";
  std::cout << "\t Current status 0x" << hex <<unsigned(*((uint8_t*)_RNOC(CMD)))<< std::endl;
#endif

  sc_dt::uint64 addr = trans.get_address();
  if (tlm::TLM_READ_COMMAND == trans.get_command()){ // Read in register
    if ( RNOC_POOLSIZE >= addr){
      memcpy(trans.get_data_ptr(), (void*)((uintptr_t)noc_register+addr), trans.get_data_length());
      trans.set_response_status(tlm::TLM_OK_RESPONSE);
    }else {
      std::cerr << name() << ": Read access error, bad address \n";
      trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    }
  }else {                                            // Write in register
    if(RNOC_CMD == addr){ //cmd
      memcpy((void*)_RNOC(CMD), trans.get_data_ptr(), trans.get_data_length());
      //trigger event
      cpuCmdEvent();
      trans.set_response_status(tlm::TLM_OK_RESPONSE);
    } else if (RNOC_POOLSIZE >= addr){
      memcpy((void*)((uintptr_t)noc_register+addr), trans.get_data_ptr(), trans.get_data_length());
      trans.set_response_status(tlm::TLM_OK_RESPONSE);
    } else{
      std::cerr << name() << ": Read access error, bad address \n";
      trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    }
  }
}

void ipc::nocItf::cpuCmdEvent(){

  switch(*((uint8_t*)_RNOC(CMD))){
  case CNOC_SORDER: {
    orderHeader order;
    memcpy(&order, (void*)_RNOC(RCVDATA),sizeof(orderHeader));
#ifdef NIPC_DEBUG_CPU
    std::cout << name() <<": SORDER request with properties:\n"
              <<" task_id"<< hex << order.task_id <<"\n"
              <<" sender_id"<< hex << order.sender_id <<"\n"
              <<" cmd"<< hex << order.cmd <<"\n"
              <<" remote"<< hex << order.remote <<"\n"
              <<" local"<< hex << order.local <<"\n"
              <<" data_len"<< hex << order.data_length <<"\n";
#endif
    //update delta packet time inside a noc sync step
    order.deltaInCycle_ns = ((uint)((sc_time_stamp().to_seconds() * 1000000000/*s_to_ns*/)
                            - (preBoot_ms*1000000/*ms_to_ns*/))% (postBoot_us *1000/*us_to_ns*/));
    //Send it on IPC socket
    int status = erFree_send(sockfd, reinterpret_cast<char*>((orderHeader *)&order), sizeof(orderHeader));
    ASSERT(sizeof(orderHeader) == status,"get "<<dec << status <<" expected "<<sizeof(orderHeader));
    if ((order.cmd != NOC_ACK)&&(order.cmd != NOC_MEMRTV)){ //if not ACK or memoryCB append order to pending list
      task_t store_id = ((MGMT_TASKID == (order.sender_id & NOC_TASKMASK))?order.task_id:order.sender_id);
      ASSERT(pendingOrder.end() == pendingOrder.find(store_id),
             "Task_id 0x"<<std::hex<<order.task_id<<" already in use, check task_id attribution\n");
      pendingOrder[store_id] = 0xff; //arm all ack flags
    }else if(order.cmd == NOC_MEMRTV){//wait a noc Sync phase and deltaInCycle delay for flushing data cb
      wait(syncEvt);
      wait(order.deltaInCycle_ns, SC_NS);
      }
    break;
  }
  case CNOC_SYNC: {
    waitPoint waitDesc;
    memcpy(&waitDesc, (void*)_RNOC(RCVDATA),sizeof(waitPoint));
    auto itOrder = pendingOrder.find(waitDesc.task_id);
    if(pendingOrder.end() != itOrder){ //order exist
      if ((itOrder->second & waitDesc.ack_type)>0){ //ack not already occur
        if(pendingWait.end() != pendingWait.find(waitDesc.task_id)) //key exist append ack flag
          pendingWait[waitDesc.task_id] |= waitDesc.ack_type;
        else // create it
          pendingWait[waitDesc.task_id] = waitDesc.ack_type;
        *((waitStatus*)_RNOC(SENDDATA)) = WAITPOINT_SET; // wait valid
      }else{
        *((waitStatus*)_RNOC(SENDDATA)) = waitDesc.ack_type; // wakeUp instant
      }
    }else{
      *((waitStatus*)_RNOC(SENDDATA)) = PRC_ACK; // wakeUp instant with default clear
    }
    break;
  }
  case CNOC_GORDER: {
    orderHeader order;
    if(rcvOrder.empty()){
      order.is_sync_header=true;
      *((uint32_t*)_RNOC(RCVORDER))= 0x00; // updt register
    }else{
      order = rcvOrder.front();
      rcvOrder.erase(rcvOrder.begin());

      // orderEvt_mgd.notify(sc_core::SC_ZERO_TIME);
      if (-1 == irqOrder_pending.trywait()){
        std::cerr <<name() <<": signal IRQ managed received but no irq pending\n";
      }else{
        *((uint32_t*)_RNOC(RCVORDER))= irqOrder_pending.get_value(); // updt register
        if(0== irqOrder_pending.get_value())
          irq_orderRcv.write(0);
        // std::cout <<name()<< ": Order :=> "<<irqOrder_pending.get_value()<<std::endl;
      }
    }
    memcpy((void*)_RNOC(SENDDATA), &order, sizeof(orderHeader));
    break;
  }
  case CNOC_WAKEUP: {
    waitPoint wakeup;
    if(wakeupTask.empty()){
      wakeup.task_id = 0;
      *((uint32_t*)_RNOC(WAKEUP))= 0x00; // updt register
    }else{
      wakeup = wakeupTask.front();
      wakeupTask.erase(wakeupTask.begin());
      wakeupDeltaInCycle.erase(wakeupDeltaInCycle.begin());
      // wakeEvt_mgd.notify(sc_core::SC_ZERO_TIME);
      if (-1 == irqWake_pending.trywait()){
        std::cerr <<name() <<": signal IRQ managed received but no irq pending\n";
      }else{
        *((uint32_t*)_RNOC(WAKEUP))= irqWake_pending.get_value(); // updt register
        if(0== irqWake_pending.get_value())
          irq_wakeUp.write(0);
        // std::cout <<name()<< ": Wake :=> "<<irqWake_pending.get_value()<<std::endl;
      }
    }
    memcpy((void*)_RNOC(SENDDATA), &wakeup, sizeof(waitPoint));
    break;
  }
  case CNOC_DISCOVER: {
    uint32_t discoverBuffer[(2*nocDiscover.size())+1];
    discoverBuffer[0] = nocDiscover.size();
    int i=1;
    for( auto & it_NoC: nocDiscover){
      discoverBuffer[i++] = (it_NoC.second.x_pos <<8) + it_NoC.second.y_pos;
      discoverBuffer[i++] = it_NoC.first;
    }
    memcpy((void*)_RNOC(SENDDATA), discoverBuffer, (1+(2*nocDiscover.size()))*sizeof(uint32_t));
    break;
  }
  case CNOC_NOCPOWER: {
    float t_noc = tns_noc * NS_TO_S;
    uint32_t transTime = *(reinterpret_cast<uint32_t*>(&t_noc)); //extract binary form of float
    uint32_t transEnj = *(reinterpret_cast<uint32_t*>(&enj_noc)); //extract binary form of float
    uint64_t powerBuffer = ((uint64_t)transEnj <<32) + transTime;
    memcpy((void*)_RNOC(SENDDATA), &powerBuffer, sizeof(uint64_t));
    break;
  }
  default:{
    std::cerr << "Error on NocItf received command unimplemented\n";
    break;
  }
  }

#ifdef NIPC_DEBUG_CPU
  std::cout << name() <<": Processed command 0x" << hex << unsigned(*((uint8_t*)_RNOC(CMD))) << "\n";
  std::cout << "\t" << "Pending Order list: \n";
  for (const auto & it_order: pendingOrder){
    std::cout << "\t \t" <<it_order.first <<"=> " << it_order.second <<" \n";
  }
  std::cout << "\t" << "Pending Wait list: \n";
  for (const auto & it_wait: pendingWait){
    std::cout << "\t \t" <<it_wait.first <<"=> " << it_wait.second <<" \n";
  }
  std::cout << "\t" << "RcvOrder list: \n";
  for (const auto & it_rcv: rcvOrder){
    std::cout << "\t \t" <<it_rcv.cmd <<"=>" << it_rcv.task_id << " \n";
  }
  std::cout << "\t" << "WakeUp list: \n";
  for (const auto & it_wakeUp: wakeupTask){
    std::cout <<std::hex << "\t \t0x"<<it_wakeUp.task_id
              <<"("<< ack_helper.find(it_wakeUp.ack_type)->second << ") \n";
  }
  std::cout << "\t" << "nocDiscover: \n";
  for( auto & it_NoC: nocDiscover){
    std::cout << "\t \t @" <<std::hex<< it_NoC.first
              << " =>["<<(int)it_NoC.second.x_pos
              << ","<<(int)it_NoC.second.y_pos << "]\n";
  }
  std::cout << std::endl;
#endif

  //reset pending cmd
  *((uint8_t*)_RNOC(CMD)) = 0x00;
}

/*---
 * NOC Router implementation
 /* ------------------------------------------------------------------------- */
ipc::nocRouter::nocRouter(int nbSk, std::string socketPath, sb::nocCnf config)
  :nocRouter_base(nbSk, config)
{
  //Alloate memory for callBack payload buffer
  cb_payload = new char[MAX_CB_PAYLOAD_LEN];
  //init unix socket for sync msg between SC instances
  if ((mainfd = socket(AF_UNIX,SOCK_STREAM,0)) < 0){
    perror("Creating router socket");
    exit(0);
  }
  mainAddr = new sockaddrUnix;
  mainAddr->sun_family = AF_UNIX;
  strcpy(mainAddr->sun_path, socketPath.c_str());
  unsigned int mainDscLen= socketPath.length()+sizeof(mainAddr->sun_family);
  if(bind(mainfd,(struct sockaddr *)mainAddr,mainDscLen)<0){
    perror("binding router socket");
    exit(0);
  }
  sockfd = new int[nbSk];
  sockAddr = new sockaddrUnix[nbSk];
  fdRs = new struct pollfd[nbSk];
  listen(mainfd,nbSk);

  //init power and time counter
  enj_log =0.0;
  tns_log = 0;

}

ipc::nocRouter::~nocRouter(){
  close(mainfd);
  delete mainAddr;
  delete sockAddr;
  delete[] sockfd;
  delete[] fdRs;
  delete[] cb_payload;
}

uint ipc::nocRouter::bindToCluster(tlm_utils::simple_target_socket<nocItf_base>* nocItf_Ssk,
                                   tlm_utils::simple_initiator_socket<nocItf_base>* nocItf_Msk,
                                   uint64_t clBaseAddr, uint8_t cl_x, uint8_t cl_y){
  static uint nxtSk=0;
  unsigned int sockLen = sizeof(sockAddr[nxtSk]);
  uint trgtSk;
  clPos cl_pos;
  ASSERT(cl_x < nocConfig.xSize,"Bad SoC configuration: Noc xSize("<<unsigned(nocConfig.xSize)
         <<") <= xPos("<<unsigned(cl_x)<<").\n");
  ASSERT(cl_y < nocConfig.ySize,"Bad SoC configuration: Noc ySize("<<unsigned(nocConfig.ySize)
         <<") <= yPos("<<unsigned(cl_y)<<").\n");

  sockfd[nxtSk] = accept(mainfd,(struct sockaddr *)&sockAddr[nxtSk],&sockLen);
  if (sockfd[nxtSk] < 0){
    perror("accepting");
    exit(0);
    nxtSk++;
  }

#ifdef NIPC_DEBUG_BIND
  std::cout << "cluster["<<unsigned(cl_x)<<", "<<unsigned(cl_y)<<"]@0x"<<hex<<clBaseAddr<<": bind on" <<dec<< nxtSk<<"\n";
#endif

  trgtSk = nxtSk++;
  cl_pos.x_pos = cl_x;
  cl_pos.y_pos = cl_y;

  clMapOnSk[clBaseAddr] = trgtSk;
  skToClpos[trgtSk] = cl_pos;
  return(trgtSk);
}

void ipc::nocRouter::startRouter(){
  uint enableSk;
  int status;
  routerEnable = true;
  while (routerEnable){
#ifdef NIPC_DEBUG_IPC
    std::cout << "======================================== ROUTER STEP ========================================\n";
#endif

    /*
     * Order phase:
     * Read each TCP stream until orderSent header; For each received order
     * forward it on right socket.
     * Broadcast orderRcv header to acknowledge this phase
     */
    orderHeader order;
    //setup read fds and Arm socket
    enableSk = ((0x1 << nb_sk)-1);
    for(int sk=0; sk < nb_sk; ++sk){
      fdRs[sk].fd = sockfd[sk];
      fdRs[sk].events = POLLIN;
    }
    do{
      status = poll(fdRs, nb_sk, NOC_POLL_TIMEOUT_MS);
      if( 0< status){ // an event occured
        for(int sk=0; sk < nb_sk; ++sk){
          if(fdRs[sk].revents &POLLIN){
            // status = recv(sockfd[sk], reinterpret_cast<char*>(&order), sizeof(orderHeader), 0);
            status = erFree_recv(sockfd[sk], reinterpret_cast<char*>(&order), sizeof(orderHeader));
            ASSERT(sizeof(orderHeader) == status,"get "<<dec << status <<" expected "<<sizeof(orderHeader));
            if(!(order.is_sync_header)) { // not a sync Header
              // update addr (Global @ to cluster local @)
              uint trgtSk;
              trgtSk = toClusterAddr((uint64_t&)order.remote);
              status = erFree_send(sockfd[trgtSk], reinterpret_cast<char*>(&order), sizeof(orderHeader));
              ASSERT(sizeof(orderHeader) == status,"get "<<dec << status <<" expected "<<sizeof(orderHeader));
              comLog(sk, trgtSk, status);
#ifdef NIPC_DEBUG_SYNC
              std::cout << "\t forward header on sk " <<dec << trgtSk << ":\n";
#endif
            } else { //Sync Header received
              enableSk &= ~(0x01 << sk); //disable sockets
              fdRs[sk].events = 0x00;
            }
          }
        }
      }
    }while(0!= enableSk);

    //Send orderSent header to all sockets and rearm channels
    order.is_sync_header = true;
    order.local = tns_log; // update powerLog on sync
    order.remote = *(reinterpret_cast<uint32_t*>(&enj_log)); //binary copy uint64 is just a transport type
    for(int sk=0; sk < nb_sk; ++sk){
      status = erFree_send(sockfd[sk], reinterpret_cast<char*>(&order), sizeof(orderHeader));
      ASSERT(sizeof(orderHeader) == status,"get "<<dec << status <<" expected "<<sizeof(orderHeader));
    }

    /*
     * data callBack phase:
     * Read each TCP stream until callbackSent header; For each received callback
     * forward it on right socket.
     * Broadcast callbackRcv header to acknowledge this phase
     */
    dcbHeader dcb;
    enableSk = ((0x1 << nb_sk)-1);
    for(int sk=0; sk < nb_sk; ++sk){
      fdRs[sk].fd = sockfd[sk];
      fdRs[sk].events = POLLIN;
    }
    do{
      status = poll(fdRs, nb_sk, NOC_POLL_TIMEOUT_MS);
      if( 0< status){ // an event occured
        for(int sk=0; sk < nb_sk; ++sk){
          if(fdRs[sk].revents &POLLIN){
            status = recv(sockfd[sk], reinterpret_cast<char*>(&dcb), sizeof(dcbHeader), 0x00);
            ASSERT(sizeof(dcbHeader) == status,"get "<<dec << status <<" expected "<<sizeof(dcbHeader));
            if(!(dcb.is_sync_header)) { // not a sync Header
#ifdef NIPC_DEBUG_SYNC
              std::cout << " Router: received callback header from " <<dec << sk << ":\n";
#endif
              //read payload
              status = erFree_recv(sockfd[sk], cb_payload, dcb.data_length);
              ASSERT(dcb.data_length == status,"get "<<dec << status <<" expected "<<dcb.data_length);

              // update addr (Global @ to cluster local @)
              uint trgtSk;
              trgtSk = toClusterAddr((uint64_t&)dcb.remote);
              status = erFree_send(sockfd[trgtSk], reinterpret_cast<char*>(&dcb), sizeof(dcbHeader));
              ASSERT(sizeof(dcbHeader) == status,"get "<<dec << status <<" expected "<<sizeof(dcbHeader));
              comLog(sk, trgtSk, status);
              status = erFree_send(sockfd[trgtSk], cb_payload, dcb.data_length);
              ASSERT(dcb.data_length == status,"get "<<dec << status <<" expected "<<dcb.data_length);
              comLog(sk, trgtSk, status);
#ifdef NIPC_DEBUG_SYNC
              std::cout << "\t forward callback header and payload on sk " <<dec << trgtSk << ":\n";
#endif
            } else { //Sync Header received
              enableSk &= ~(0x01 << sk); //disable sockets
              fdRs[sk].events = 0x00;
            }
          }
        }
      }
    }while(0!= enableSk);

    //Send callbackSent header to all sockets and rearm channels
    dcb.is_sync_header = true;
    dcb.remote= 0xffff;
    for(int sk=0; sk < nb_sk; ++sk){
      status = erFree_send(sockfd[sk], reinterpret_cast<char*>(&dcb), sizeof(dcbHeader));
      ASSERT(sizeof(dcbHeader) == status,"get "<<dec << status <<" expected "<<sizeof(dcbHeader));
    }
  }
}
