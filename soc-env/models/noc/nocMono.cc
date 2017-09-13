#include "nocMono.h"
using namespace noc;

/*---
 * NOC Interfaces implementation
 * ------------------------------------------------------------------------- */
mono::nocItf::nocItf(sc_core::sc_module_name name, std::map<uint, noc::clPos>nocDiscover)
  :noc::nocItf_base(name, nocDiscover), nocItf_Ssk("nocItf_Ssk"), nocItf_Msk("nocItf_Msk")
{
  // register tlm callback
  nocItf_Ssk.register_b_transport(this, &mono::nocItf::b_noc_transport);
}

mono::nocItf::~nocItf() {
}

tlm_utils::simple_target_socket<nocItf_base>* mono::nocItf::get_noc_Ssk(){
  return (reinterpret_cast<tlm_utils::simple_target_socket<nocItf_base>*>(&nocItf_Ssk));
}
tlm_utils::simple_initiator_socket<nocItf_base>* mono::nocItf::get_noc_Msk(){
  return (reinterpret_cast<tlm_utils::simple_initiator_socket<nocItf_base>*>(&nocItf_Msk));
}

void mono::nocItf::b_noc_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
{
#ifdef NMONO_DEBUG_TLM
  std::cout << "@ "<< sc_time_stamp() << " => " << name() << " TLM Transaction received \n";
  std::cout << "Transaction properties:\n";
  std::cout << "\t addr: 0x"<<hex<< trans.get_address()<<"\n";
  std::cout << "\t CMD: 0x"<< hex<< trans.get_command() <<"\n";
  std::cout << "\t length:"<<dec << trans.get_data_length() <<"\n";
  std::cout << "\t Current status 0x" << hex <<unsigned(*((uint8_t*)_RNOC(CMD)))<< std::endl;
#endif

  if (tlm::TLM_READ_COMMAND == trans.get_command()){ // NOC_MEMRTV encoded as native TLM request
    //NOC_MEMRTV cmd directly encapsulated in TLM transaction
    DMA_Msk->b_transport(trans, delay);
  }else { // Write cmd use orderHeader packaging legacy of Nipc implementation
    orderHeader order;
    order = *((orderHeader*)trans.get_data_ptr());
    switch(order.cmd){
    case NOC_ACK: {
#ifdef NMONO_DEBUG_TLM
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
        if(PRC_ACK == order.local) //Clear on PRC but keep NACK
          pendingOrder.erase(itOrder); //delete entry
        else
          itOrder->second &= ~order.local;
      }else{
#ifdef NMONO_DEBUG_TLM
        std::cerr << name()<< ": Error received an ack: "<< ack_helper.find(order.local)->second
                  <<" for unknow task_id, or already process \n";
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
          *((uint32_t*)_RNOC(WAKEUP))= wakeupTask.size(); // updt register
          irq_wakeUp.write(1);

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
#ifdef NMONO_DEBUG_TLM
      std::cout<< "NOC_JOBASK || NOC_JOBSYNC ||NOC_MEMGET Order \n";
#endif
      rcvOrder.push_back(order);
      *((uint32_t*)_RNOC(RCVORDER))= rcvOrder.size(); // updt register
      irq_orderRcv.write(1);
      break;
    }
    default:{
      std::cerr << sc_time_stamp()<< name()<< "received write request with bad order command\n";
        break;
    }
    }

  }
}

void mono::nocItf::b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
  {
#ifdef NMONO_DEBUG_TLM
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
        cpuCmdEvent(); //trigger event
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

void mono::nocItf::cpuCmdEvent(){
    tlm::tlm_generic_payload trans;

    switch(*((uint8_t*)_RNOC(CMD))){
    case CNOC_SORDER: {
      orderHeader order;
      memcpy(&order, (void*)_RNOC(RCVDATA),sizeof(orderHeader));
#ifdef NMONO_DEBUG_CPU
      std::cout << name() <<": SORDER request with properties:\n"
                <<" task_id"<< hex << order.task_id <<"\n"
                <<" cmd"<< hex << order.cmd <<"\n"
                <<" remote"<< hex << order.remote <<"\n"
                <<" local"<< hex << order.local <<"\n"
                <<" data_len"<< hex << order.data_length <<"\n";
#endif
      if (NOC_MEMRTV == order.cmd){
        // "IPC legacy": No callback packet in this implementation.
        // Use native TLM request to mimic the mechanism
        // return adress encoded in nocAddr space at end of data
        // router return data with return adress update to localAddr space
        // => write data in local memory with new TLM packet
        // Dirty here to mimic IPC legacy mechanism

        //1: get data and local Addr from router
        char *tmp = (char*) malloc(order.data_length*sizeof(char)+sizeof(uint64_t));
        *(uint64_t*)(tmp +order.data_length) = order.local;
        trans.set_address(order.remote);
        trans.set_command(tlm::TLM_READ_COMMAND);
        trans.set_data_length(order.data_length);
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(tmp));
        //update time and send order
        sc_core::sc_time time(order.deltaInCycle_ns, SC_NS);
        nocItf_Msk->b_transport(trans,time);
        //2: write data in local memory at update Addr
        trans.set_address(*((uint64_t*)(tmp+(order.data_length))));
        trans.set_command(tlm::TLM_WRITE_COMMAND);
        trans.set_data_length(order.data_length);
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(tmp));
        DMA_Msk->b_transport(trans, time);
        free(tmp);

      }else{ //encapsulate order in TLM trans (legacy of Nipc implementation)
        trans.set_address(order.remote);
        trans.set_command(tlm::TLM_WRITE_COMMAND);
        trans.set_data_length(sizeof(orderHeader));
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(&order));
        if (order.cmd != NOC_ACK){ //if not ACK append order to pending list
            ASSERT(pendingOrder.end() == pendingOrder.find(order.task_id),
                   "Task_id already in use, check task_id attribution\n");
            pendingOrder[order.task_id] = 0xff; //arm all ack flags
          }
        //update time and send order
        sc_core::sc_time time(order.deltaInCycle_ns, SC_NS);
        nocItf_Msk->b_transport(trans,time);
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
      irq_orderRcv.write(0); // reset irq_orderRcv line
      orderHeader order;
      if(rcvOrder.empty()){
        order.is_sync_header=true;
        *((uint32_t*)_RNOC(RCVORDER))= 0x00; // updt register
      }else{
        order = rcvOrder.front();
        rcvOrder.erase(rcvOrder.begin());
        if(!(rcvOrder.empty())){
          irq_orderRcv.write(1); // rearm irq line
        }
        *((uint32_t*)_RNOC(RCVORDER))= rcvOrder.size(); // updt register
      }
      memcpy((void*)_RNOC(SENDDATA), &order, sizeof(orderHeader));
      break;
    }
    case CNOC_WAKEUP: {
      irq_wakeUp.write(0); // reset irq wakeup line
      waitPoint wakeup;
      if(wakeupTask.empty()){
        wakeup.task_id = 0;
        *((uint32_t*)_RNOC(WAKEUP))= 0x00; // updt register
      }else{
        wakeup = wakeupTask.front();
        wakeupTask.erase(wakeupTask.begin());
        if(!(wakeupTask.empty())){
          irq_wakeUp.write(1); // rearm irq line
        }
        *((uint32_t*)_RNOC(WAKEUP))= wakeupTask.size(); // updt register
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
      uint32_t transEnj = *(reinterpret_cast<uint32_t*>(&enj_noc)); //extract binary form of float
      uint64_t powerBuffer = ((uint64_t)transEnj <<32) + tns_noc;
      memcpy((void*)_RNOC(SENDDATA), &powerBuffer, sizeof(uint64_t));
      break;
    }
    default:{
      std::cerr << "Error on NocItf received command unimplemented\n";
      break;
    }
    }

#ifdef NMONO_DEBUG_CPU
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
mono::nocRouter::nocRouter(sc_core::sc_module_name name, int nbSk, sb::nocCnf config)
  : nocRouter_base(nbSk, config), sc_module(name)
  {
    //Create master and slave sockets for each cluster
    noc_Ssk = new tlm_utils::simple_target_socket_tagged<nocRouter>*[nbSk];
    noc_Msk = new tlm_utils::simple_initiator_socket<nocRouter>*[nbSk];

    //init name and bind them with clusters
    for(uint sk=0; sk< nbSk; sk++){
      std::string skEntry = "cluster_"+std::to_string(sk)+"_entry";
      noc_Ssk[sk] = new tlm_utils::simple_target_socket_tagged<nocRouter>(skEntry.c_str());
      noc_Ssk[sk]->register_b_transport(this, &mono::nocRouter::b_transport, sk);
      std::string skCallback = "cluster_"+std::to_string(sk)+"_cb";
      noc_Msk[sk] = new tlm_utils::simple_initiator_socket<nocRouter>(skCallback.c_str());
    }

    //init power and time counter
    enj_log =0.0;
    tns_log = 0;
  }

mono::nocRouter::~nocRouter(){
    for (uint sk=0 ; sk< nb_sk; sk++){
      delete noc_Ssk[sk];
      delete noc_Msk[sk];
    }
    delete[] noc_Ssk;
    delete[] noc_Msk;
  }

uint mono::nocRouter::bindToCluster(tlm_utils::simple_target_socket<nocItf_base>* nocItf_Ssk,
                                    tlm_utils::simple_initiator_socket<nocItf_base>* nocItf_Msk,
                                    uint64_t clBaseAddr, uint8_t cl_x, uint8_t cl_y){
    static uint nxtSk=0;
    uint trgtSk;
    clPos cl_pos;
    ASSERT(cl_x < nocConfig.xSize,"Bad SoC configuration: Noc xSize("<<unsigned(nocConfig.xSize)
           <<") <= xPos("<<unsigned(cl_x)<<").\n");
    ASSERT(cl_y < nocConfig.ySize,"Bad SoC configuration: Noc ySize("<<unsigned(nocConfig.ySize)
           <<") <= yPos("<<unsigned(cl_y)<<").\n");

#ifdef NMONO_DEBUG_BIND
    std::cout << "cluster["<<unsigned(cl_x)<<", "<<unsigned(cl_y)<<"]@0x"<<hex<<clBaseAddr<<": bind on" <<dec<< nxtSk<<"\n";
#endif

    trgtSk = nxtSk++;
    cl_pos.x_pos = cl_x;
    cl_pos.y_pos = cl_y;

    clMapOnSk[clBaseAddr] = trgtSk;
    skToClpos[trgtSk] = cl_pos;

    //bind tlm socket together
    noc_Msk[trgtSk]->bind(*nocItf_Ssk);
    nocItf_Msk->bind(*noc_Ssk[trgtSk]);

    return(trgtSk);
  }

void mono::nocRouter::b_transport(int id, tlm::tlm_generic_payload& trans, sc_time& delay){
#ifdef NMONO_DEBUG_TLM
    std::cout << "@ "<< sc_time_stamp() << " => " << name() << " TLM Transaction received \n";
    std::cout << "Transaction properties:\n";
    std::cout << "\t addr: 0x"<<hex<< trans.get_address()<<"\n";
    std::cout << "\t CMD: 0x"<< hex<< trans.get_command() <<"\n";
    std::cout << "\t length:"<<dec << trans.get_data_length() <<std::endl;
#endif

    uint trgtSk;
    if (tlm::TLM_READ_COMMAND == trans.get_command()){ // NOC_MEMRTV
      // native TLM transaction are use to mimic nocIpc callback feature
      uint64_t remote, local;
      remote = trans.get_address();
      trgtSk = toClusterAddr((uint64_t&)remote);
      trans.set_address(remote);
      //update return local addr
      local = *((uint64_t*)(trans.get_data_ptr()+trans.get_data_length()));
      toClusterAddr((uint64_t&)local);
      *((uint64_t*)(trans.get_data_ptr()+trans.get_data_length())) = local;

    }else { // Write => orderHeader encapsulated here
      ASSERT(sizeof(orderHeader) == trans.get_data_length(),"nocRouter received invalid transaction "
             <<dec <<trans.get_data_length() <<" expected orderHeader:"<<sizeof(orderHeader));
      //update orderHeader remote addr and get output socket
      orderHeader order;
      order = *((orderHeader*)trans.get_data_ptr());
      trgtSk = toClusterAddr((uint64_t&)order.remote);
      trans.set_address(order.remote);
    }
    (*noc_Msk[trgtSk])->b_transport(trans, delay);
    comLog(id, trgtSk, trans.get_data_length());
  }
