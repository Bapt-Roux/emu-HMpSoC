/* ---
 *  This file define an API for working thread in clusters
 *
 *  Author <baptiste.roux AT inria.fr>
 * -------------------------------------------------------------------------- */

#include "clMgmtAPI.h"

using namespace clmgmt;
using namespace napi;
using namespace std;


/* ---
 * intra-Cluster communications
 * provide memory sharing facilities for zero copy communications
 * -------------------------------------------------------------------------- */
int8_t icCom::lMemGive(subPrc_t comItf, noc::task_t task_id, void *mem, size_t size){
  localHeader_t order;

  order.trgt = task_id;
  order.size = size;
  //get memory blk properties
  monitored_getBlkInfo(mem, &order.blk_head, &order.offset);

  size_t sendBytes=0;
  int nBytes;
  while(sendBytes < sizeof(localHeader_t)){
    nBytes = write(comItf.wPipe, &order + sendBytes, sizeof(localHeader_t)-sendBytes);
    if(0> nBytes){
      cerr << __PRETTY_FUNCTION__ << ": Write Pipe error \n";
      return nBytes;
    }else{
      sendBytes += nBytes;
    }
  }
  print_DMGMT("sentOrder: \n trgt 0x"<<order.trgt
              << ", head_blkAddr 0x "<< order.blk_head
              << ", rOffset 0x "<< order.offset<<"\n");
  return 0;
}

void *icCom::b_lMemAcquire(subPrc_t comItf, noc::task_t task_id, size_t &size){
  localHeader_t rcvOrder;

  size_t rcvBytes=0;
  int nBytes;
  void* ptr;
  while(rcvBytes < sizeof(localHeader_t)){
    nBytes = read(comItf.rPipe, &rcvOrder + rcvBytes, sizeof(localHeader_t)-rcvBytes);
    if(0> nBytes){
      cerr << __PRETTY_FUNCTION__ << ": Write Pipe error \n";
      return NULL;
    }else{
      rcvBytes += nBytes;
    }
  }
  print_DMGMT("rcvOrder: \n trgt 0x"<<rcvOrder.trgt
              << ", head_blkAddr 0x "<< rcvOrder.blk_head
              << ", rOffset 0x "<< rcvOrder.offset<<"\n");
  //check match task_id
  // if ( rcvOrder.trgt != task_id){
  //   cerr << __PRETTY_FUNCTION__ << ": Target Id error \n"
  //        <<"get 0x"<<hex << rcvOrder.trgt
  //        << " instead of 0x"<< task_id << "\n";
  //   return NULL;
  // } //TODO check validity from prc provider and enable multilocal communication.
  size = rcvOrder.size;

  // shared a subpart of the memory blk
  ptr = monitored_shared(rcvOrder.blk_head, rcvOrder.offset, rcvOrder.size);
  return ptr;
}


/* ---
 * intra-Cluster router
 * manage a list of spawn process and route message between them
 * -------------------------------------------------------------------------- */
icRouter::icRouter(uint nbTrigger)
  :slavesTrigger(nbTrigger)
{
  pthread_mutex_init(&lock, NULL);
  pthread_cond_init(&event_sig, NULL);
  kill = false;
}
icRouter::~icRouter(){
  pthread_mutex_destroy(&lock);
  pthread_cond_destroy(&event_sig);
}
void icRouter::addChild(noc::task_t tsk, subPrc_t sPrc){
      childPool[tsk] = sPrc; // save child properties
      pthread_cond_signal(&event_sig); // notify router
    }
void icRouter::routerWaitRun(){
      pthread_mutex_lock(&lock);
      while(childPool.size()<slavesTrigger){pthread_cond_wait(&event_sig, &lock);}
      pthread_mutex_unlock(&lock);
    }
void icRouter::updtChild(){
      pid_t returnPrc;
      do{
        returnPrc = waitpid(0, NULL, WNOHANG);
        if(returnPrc>0){
          pthread_mutex_lock(&lock);
          for(auto inMap:childPool){
            if(inMap.second.pid == returnPrc)
              childPool.erase(childPool.find(inMap.first));
          }
          pthread_mutex_unlock(&lock);
        }
      }while(returnPrc !=0);
    }
void icRouter::stopRouter(){kill = true;}

subPrc_t icRouter::getSubPrcOf(noc::task_t tsk){
  auto trgt = childPool.find(tsk);
  if (childPool.end() == trgt){
    cerr << __PRETTY_FUNCTION__ << ": Tast unknow 0x" << std::hex <<tsk<<".\n";
    subPrc_t err;
    err.pid = -1;
    return err;
  }else{
    return trgt->second;
  }
}

void icRouter::routeMsg(void *args){
  ushort nbChild;
  localHeader_t tmpOrder;
  struct pollfd *fds;
  int status;
  uint totBytes=0;
  int nBytes;
  icRouter *pool = (icRouter*) args;

  while( !pool->kill){
    pool->routerWaitRun();

    // init Fds
    nbChild = pool->childPool.size();
    fds = (struct pollfd*) malloc(nbChild*sizeof(struct pollfd));
    int timeout_msec = PIPE_TIMEOUT_MS;
    auto itChild = pool->childPool.begin();
    for(ushort i=0; i< nbChild; i++){
      fds[i].fd = itChild->second.rPipe;
      fds[i].events = POLLIN;
      itChild++;
    }
    while(pool->childPool.size() == nbChild && !pool->kill){ // while Fds valid
      //poll pipe
      status = poll(fds, nbChild, timeout_msec);
      if( 0< status){ // an event occured
        for(ushort i=0; i< nbChild; i++){
          if(fds[i].revents & POLLIN){
            totBytes=0;
            while(totBytes < sizeof(localHeader_t)){
              nBytes = read(fds[i].fd, &tmpOrder + totBytes, sizeof(localHeader_t)-totBytes);
              if(0> nBytes){
                cerr << __PRETTY_FUNCTION__ << ": Read Pipe error \n";
                continue;
              }else{totBytes += nBytes;}
            }
            print_DMGMT("Order to route: \n trgt 0x"<<tmpOrder.trgt
                        << ", head_blkAddr 0x "<< tmpOrder.blk_head
                        << ", rOffset 0x "<< tmpOrder.offset<<"\n");
            //find trgt
            auto trgt = pool->childPool.find(tmpOrder.trgt);
            if (pool->childPool.end() == trgt){
              cerr << __PRETTY_FUNCTION__ << ": Trgt unknow 0x" << std::hex <<tmpOrder.trgt<<".\n";
              continue;
            }else{
              totBytes=0;
              while(totBytes < sizeof(localHeader_t)){
                nBytes = write(trgt->second.wPipe, &tmpOrder + totBytes, sizeof(localHeader_t)-totBytes);
                if(0> nBytes){
                  cerr << __PRETTY_FUNCTION__ << ": Write Pipe error \n";
                  continue;
                }else{totBytes += nBytes;}
              }
            }
          }
        }
      }
      //check if subprc have finished here ?
      pool->updtChild();
    }
    free(fds);
  }
}

/* ---
 * Cluster Management base
 * provide shared method for master and endPoint
 * Spawn thread for icRouter execution
 * -------------------------------------------------------------------------- */
clMBase::clMBase(std::string dev, uint routerTrigger)
  : nocdev(dev), localRouter(routerTrigger)
{
  //spawn router pthread
  pthread_create(&localRouter.pid, NULL, (void * (*)(void *))icRouter::routeMsg, &localRouter);
}

clMBase::~clMBase(){
  // ask router to stop and wait for it
  localRouter.stopRouter();
  // pthread_join(localRouter.pid, NULL);
}

/* ---
 * helper for getting and reseting cluster power management unit
 * TODO use device-tree value to obtain the PM address
 */
void clMBase::resetCPM(){
  void *cpm;
  int fd;
	fd = open ("/dev/mem", O_RDWR);
	if (fd < 1) {
    cerr<<"Error: opening /dev/mem for power monitoring management\n";
    return;
	}
  cpm = mmap(NULL, 0x8, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x5fff0000);
  // Write in register to force reset
  *((float*) cpm) = 0.0;
  *((float*) cpm+1) = 0.0;
  print_DMGMT("Power values after reset: [eng:"<< *((float*) cpm)<<"; time:"<<*((float*) cpm+1)<<"]\n");
  munmap(cpm, 0x08);

  return;
}
void clMBase::getCPM(float &eng, float &time){
  void *cpm;
  int fd;
	fd = open ("/dev/mem", O_RDWR);
	if (fd < 1) {
    cerr<<"Error: opening /dev/mem for power monitoring management\n";
    return;
	}
  cpm = mmap(NULL, 0x8, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x5fff0000);
  eng = *((float*) cpm);
  time = *((float*) cpm+1);
  munmap(cpm, 0x08);

  print_DMGMT("Read power values: [eng:"<< eng <<"; time:"<<time<<"]\n");
  return;
}

int8_t clMBase::spawnChild(napi::trgtId_t parent){
  int8_t status = 0;
  //create rPipe and wPipe for child communication
  int chWrite[2];
  pipe(chWrite);
  int chRead[2];
  pipe(chRead);

  //fork process for child
  pid_t child_pid = fork();
  if(0 == child_pid){
    { // this extra scope force local object destruction befor exit call
      worker_env env;
      env.wPipe = chWrite;
      env.rPipe = chRead;
      env.nocDevice = nocdev;
      env.parent = parent;
      print_DMGMT("Launch worker 0x"<<hex<< (parent.first & NOC_TASKMASK)
                  <<" on prc 0x"<<getpid()<<".\n");
      clSlave runner(env);
    };
    exit(0);

  }else if(0> child_pid) {
    cerr << "Fork failed for worker : "<< parent.first
         <<" ask by cluster @0x"<<hex<<parent.second<< endl;
    status=child_pid;
  }else{ /*Parents process*/
    subPrc_t child;
    child.pid = child_pid;
    spawnedChild.push_back(child_pid);
    // close unused pipe end
    close(chRead[0]);
    child.wPipe = chRead[1];
    close(chWrite[1]);
    child.rPipe = chWrite[0];
    localRouter.addChild(parent.first, child);
  }
  return(status);
}

/* ---
 * Cluster Master
 * -------------------------------------------------------------------------- */
clMaster::clMaster(const std::string& device):
  clMBase(device, 2), nocApi(device)
{
  resetCPM();
  getNocApi().get_powerNoc(sNocEng, sNocTime);
  print_DMGMT("Noc PM value @Start: [Energy "<<sNocEng<<"; Time "<<sNocTime <<"]\n");

}

clMaster::~clMaster(){
  // wait pending child
  for(auto child: spawnedChild){
    std::cout << "Wait Child: "<< child<< "\n";
    waitpid(child, NULL, 0);
    std::cout << "Child: "<< child<< " returned \n\n";
  }
  float tmpEng, tmpTime;
  getCPM(tmpEng, tmpTime);
  std::cout << " MASTER POWER MONITORING COUNTER ------------------------\n";
  std::cout << "Cluster PM value: [Energy "<<tmpEng<<"; Time "<<tmpTime <<"]\n";
  getNocApi().get_powerNoc(tmpEng, tmpTime);
  std::cout<< "Noc PM value: [Energy "<<(tmpEng - sNocEng)<<"; Time "<<(tmpTime -sNocTime) <<"]\n";
  std::cout << "--------------------------------------------------\n";
}

int8_t clMaster::askSlaveSubscribe(trgtId_t &trgt){

  if (trgt.second == nocApi.getClAddr()){ //local worker
    print_DMGMT("Spawn local worker: no register entry in Noc\n");
    return(spawnChild(trgt));
  }else{
    print_DMGMT("Ask for remote worker\n");
    return(nocApi.askSlaveSubscribe(trgt));
  }
}

void clMaster::setProviderAndReceiver(trgtId_t trgt, deque<comProps> providers, deque<comProps> receivers){
  ushort nbProviders = providers.size();
  ushort nbReceivers = receivers.size();
  ushort bufferSize = (sizeof(uint32_t) + (nbProviders+nbReceivers)*sizeof(comProps));
  void* rawPtr;
  void* rawBuffer = monitored_malloc(bufferSize);

#ifdef CLMGMT_DEBUG
  std::cout << __PRETTY_FUNCTION__<<": Sent data:\n";
  std::cout <<"\t Providers: ["<<providers.size()<<"]\n";
  for(auto it:providers){
    std::cout <<"\t \t Task 0x"<<std::hex<< it.task
              <<", cl@0x"<< it.clAddr <<" dSize 0x"<<it.dSize <<"\n";
  }
  std::cout <<"\t Receivers: ["<<receivers.size()<<"]\n";
  for(auto it:receivers){
    std::cout <<"\t \t Task 0x"<<std::hex<< it.task
              <<", cl@0x"<< it.clAddr <<" dSize 0x"<<it.dSize <<"\n";
  }
#endif

  //Put vector data in rawBuffer
  *((uint32_t*)rawBuffer) = (uint32_t)((nbProviders << 16) + nbReceivers);

  //cast providers vector
  rawPtr = (void*)((uintptr_t)rawBuffer+sizeof(uint32_t));
  for(int i=0; i<nbProviders; i++){
    *((comProps*)rawPtr +i) = providers.front();
    providers.pop_front();
  }

  //build receiver vector
  rawPtr = (void*)((uintptr_t)rawBuffer+(sizeof(uint32_t) + nbProviders*sizeof(comProps)));
  for(int i=0; i<nbReceivers; i++){
    *((comProps*)rawPtr +i) = receivers.front();
    receivers.pop_front();
  }

  //send data
  if (trgt.second == nocApi.getClAddr()){ //local communication
    icCom::lMemGive( localRouter.getSubPrcOf(trgt.first), trgt.first, rawBuffer, bufferSize);
  }else{
    nocApi.b_sendData(trgt, MGMT_TASKID, rawBuffer, bufferSize);
  }

  monitored_free(rawBuffer);
  return;
}

int8_t clMaster::b_sendData(napi::trgtId_t trgt, void* data, size_t data_length){
  int8_t status;
  if (trgt.second == nocApi.getClAddr()){ //local communication
    status = icCom::lMemGive( localRouter.getSubPrcOf(trgt.first), trgt.first, data, data_length);
  }else{
    status = nocApi.b_sendData(trgt, trgt.first, data, data_length);
  }
  return status;
}

size_t clMaster::b_rcvData(napi::trgtId_t trgt, void* &buffer, size_t maxSize){
  size_t size;
  if (trgt.second == nocApi.getClAddr()){ //local communication
    if(NULL != buffer)
      monitored_free(buffer);
    buffer = icCom::b_lMemAcquire( localRouter.getSubPrcOf(trgt.first), trgt.first, size);
  }else{
    if(NULL == buffer)
      buffer = monitored_malloc(maxSize);
    size = nocApi.b_rcvData( trgt.first, MGMT_TASKID, buffer, maxSize);
  }
  return size;
}


/* ---
 * Cluster EndPoint
 * -------------------------------------------------------------------------- */
clEndPoint::clEndPoint(const std::string& device):
  clMBase(device, 2), nocApi(std::make_pair(MGMT_TASKID, 0x00),device)
{
  resetCPM();
  nocEndPoint(); // stmt never return
}

clEndPoint::~clEndPoint(){
}

/* ---
 * Listen noc JOB_ASK message, answer and spawn corresponding child
 *    start and manage localRouter in a sub-thread when necessary
 * ---*/
void clEndPoint::nocEndPoint(){
  int8_t status;
  while(1){ //endPoint is an ad libitum mode
    noc::orderHeader rcvJob;
    rcvJob = nocApi.ep_waitJob();
    //no policy yet accept all
    // FIXME: accept only if worker available localy.
    status = spawnChild(std::make_pair(rcvJob.task_id, rcvJob.local));
    if(0> status){ //send NACK on error else PRC_ACK send by child
      trgtId_t request = make_pair(rcvJob.task_id, rcvJob.local);
      nocApi.job_answer(request, NACK);
    }
  }
}

/* ---
 * Cluster Slave
 * -------------------------------------------------------------------------- */
// clSlave::clSlave(trgtId_t mId, int wPipe[2], int rPipe[2],
//                  const std::string& device):
clSlave::clSlave(worker_env env):
  nocApi(env.parent, env.nocDevice),
  slave_id(env.parent.first)
{
  // get parents pid => clMaster
  masterPrc.pid = getppid();
  // close unused pipe end
  close(env.wPipe[0]);
  masterPrc.wPipe = env.wPipe[1];
  close(env.rPipe[1]);
  masterPrc.rPipe = env.rPipe[0];

  // startWorker();
  startSamplesWorker(); //for powerTest only
}

clSlave::~clSlave(){
  float tmpEng, tmpTime;
  clMBase::getCPM(tmpEng, tmpTime);
  std::cout << " SLAVES POWER MONITORING COUNTER UPDATE ------------------------\n";
  std::cout << "Cluster PM value: [Energy "<<tmpEng<<"; Time "<<tmpTime <<"]\n";
  std::cout << "----------------------------------------------------------------\n";

  //close end of pipe
  close(masterPrc.wPipe);
  close(masterPrc.rPipe);
}

void clSlave::startWorker(){
  if(nocApi.getParent().second != nocApi.getClAddr()){
  //send ack to master over noc
  nocApi.job_answer(nocApi.getParent(), PRC_ACK);
  }

  getProviderAndReceiver();
  bool localAlloc =false;
  void * dataIn; //input data serialize in array
  void * dataOut; // output data ptr: worker allocate output data
                  //    or point to a portion of input one.
  // Retrieve data
  if(1 == providersList.size()){ //one provider no data origin check
    comProps prvd = providersList.front();
    providersList.pop_front();
    size_t dSize=0;
    if(nocApi.getClAddr() == prvd.clAddr){ //Local com
      dataIn = icCom::b_lMemAcquire(masterPrc, prvd.task, dSize);
    }else{
      dataIn = monitored_malloc(prvd.dSize);
      dSize = nocApi.b_rcvData(slave_id, prvd.task, dataIn, prvd.dSize);
    }
    if(dSize != prvd.dSize){
      cerr << "Provider dSize and real received size mismatch\n";
    }
#ifdef DATA_DUMP_DEBUG
    std::cout << " Inputs data received: \n";
    for(uint c=0; c<prvd.dSize; c++)
      std::cout << "["<<std::dec <<c<<"] => "<< (uint) ((uint8_t*)dataIn)[c] <<"\n";
#endif
  }else if(1 < providersList.size()){ //multiple prv need mem realign on local access and origin check
    //FIXME:
    cerr << "MULTI PROVIDERS NOT IMPLEMENTED YET";
    exit(-1);
  }// else no input


  std::cout << "Task 0x"<<std::hex<<(slave_id& NOC_JOBMASK)<<" started \n";
  std::cout << "\t dataIn: 0x"<<std::hex << (uintptr_t) dataIn <<"\n"
            << "\t dataOut: 0x"<<std::hex << (uintptr_t) dataOut <<"\n";
  (machSuiteWorker.at(slave_id& NOC_JOBMASK))(dataIn, dataOut, localAlloc);
  std::cout << "Task 0x"<< std::hex<<(slave_id& NOC_JOBMASK)<<" end \n";
  std::cout << "\t dataIn: 0x"<<std::hex << (uintptr_t) dataIn <<"\n"
            << "\t dataOut: 0x"<<std::hex << (uintptr_t) dataOut <<"\n";

  // Send data
  if(1 == receiversList.size()){ //one receiver
    comProps rcvr = receiversList.front();
    receiversList.pop_front();
    if(nocApi.getClAddr() == rcvr.clAddr){ //Local com
      icCom::lMemGive(masterPrc, rcvr.task, dataOut, rcvr.dSize);
    }else{
      nocApi.b_sendData(make_pair(rcvr.task, rcvr.clAddr), slave_id, dataOut, rcvr.dSize);
    }
#ifdef DATA_DUMP_DEBUG
    std::cout << " Outputs data sent: \n";
    for(uint c=0; c<rcvr.dSize; c++)
      std::cout << "["<<std::dec <<c<<"] => "<< (uint) ((uint8_t*)dataOut)[c] <<"\n";
#endif
  }else if(1 < receiversList.size()){ //multiple prv need mem realign on local access and origin check
    //FIXME:
    cerr << "MULTI RECEIVERS NOT IMPLEMENTED YET";
    exit(-1);
  }// else no output

  monitored_free(dataIn);
  if(localAlloc){monitored_free(dataOut);}
  return;
}
void clSlave::startSamplesWorker(){
  if(nocApi.getParent().second != nocApi.getClAddr()){
  //send ack to master over noc
  nocApi.job_answer(nocApi.getParent(), PRC_ACK);
  }

  getProviderAndReceiver();
  bool localAlloc =false;
  void * dataIn = NULL; //input data serialize in array
  void * dataOut = NULL; // output data ptr: worker allocate output data
                  //    or point to a portion of input one.
  void* valid = NULL; //valid output => not used
  size_t lIn, lOut;

  // Retrieve data
  for(auto prvd:providersList){
      std::cout <<"\t get Data from Task 0x"<<std::hex<< prvd.task
                <<", cl@0x"<< prvd.clAddr <<" dSize 0x"<<prvd.dSize <<"\n";
    if(NULL != dataIn){monitored_free(dataIn);}
    size_t dSize=0;
    if(nocApi.getClAddr() == prvd.clAddr){ //Local com
      dataIn = icCom::b_lMemAcquire(masterPrc, prvd.task, dSize);
    }else{
      dataIn = monitored_malloc(prvd.dSize);
      dSize = nocApi.b_rcvData(slave_id, prvd.task, dataIn, prvd.dSize);
    }
    if(dSize != prvd.dSize){
      cerr << "Provider dSize and real received size mismatch\n";
    }
  }

  // Allocate buffer and setup sample data
  if(NULL != dataIn){monitored_free(dataIn);}
  (machSuiteGetSamples.at(slave_id&(NOC_JOBMASK^HWFLAG)))(dataIn, lIn, dataOut, lOut, valid);
  if(NULL!= valid){monitored_free(valid);} //monitored_free unused buffer
  if(NULL!= dataOut){monitored_free(valid);}

  std::cout << "Task 0x"<<std::hex<<(slave_id& NOC_JOBMASK)<<" started \n";
  (machSuiteWorker.at(slave_id& NOC_JOBMASK))(dataIn, dataOut, localAlloc);
  std::cout << "Task 0x"<< std::hex<<(slave_id& NOC_JOBMASK)<<" end \n";

  // Send data
  for(auto rcvr:receiversList){
    std::cout <<"\t send Data to Task 0x"<<std::hex<< rcvr.task
              <<", cl@0x"<< rcvr.clAddr <<" dSize 0x"<<rcvr.dSize <<"\n";
    if(nocApi.getClAddr() == rcvr.clAddr){ //Local com
      icCom::lMemGive(masterPrc, rcvr.task, dataOut, rcvr.dSize);
    }else{
      nocApi.b_sendData(make_pair(rcvr.task, rcvr.clAddr), slave_id, dataOut, rcvr.dSize);
    }
  }

  monitored_free(dataIn);
  if(localAlloc){monitored_free(dataOut);}
  return;
  }

  void clSlave::getProviderAndReceiver(){
    ushort nbProviders;
    ushort nbReceivers;
    size_t bufferSize = (sizeof(uint32_t) + (MAX_PROVIDERS+MAX_RECEIVER)*sizeof(comProps));
    size_t rcvSize;
    void* rawPtr;
    void* rawBuffer;

    if(nocApi.getParent().second != nocApi.getClAddr()){
      rawBuffer = monitored_malloc(bufferSize);
      rcvSize = nocApi.b_rcvData(slave_id, MGMT_TASKID, rawBuffer, bufferSize);//get raw data from Noc
    }else{
      rawBuffer = icCom::b_lMemAcquire(masterPrc, slave_id, rcvSize);
    }

    nbProviders = *((uint32_t*)rawBuffer)>>16;
    nbReceivers = *((uint32_t*)rawBuffer)&0xffff;
    print_DMGMT("rcv size:"<<rcvSize<< ", firstSlot: "<<sizeof(uint32_t) + nbProviders*sizeof(comProps)<<"\n");
    //build providers vector
    rawPtr = (void*)((uintptr_t)rawBuffer+sizeof(uint32_t));
    for(int i=0; i<nbProviders; i++){
      comProps buffer;
      buffer = *((comProps*)rawPtr +i);
      providersList.push_back(buffer);
    }

    //build receiver vector
    rawPtr = (void*)((uintptr_t)rawBuffer+(sizeof(uint32_t) + nbProviders*sizeof(comProps)));
    for(int i=0; i<nbReceivers; i++){
      comProps buffer;
      buffer = *((comProps*)rawPtr +i);
      receiversList.push_back(buffer);
    }
#ifdef CLMGMT_DEBUG
    std::cout << __PRETTY_FUNCTION__<<": Received data: 0x"<<std::hex<<*((uint32_t*)rawBuffer)<<"\n";
    std::cout <<"\t Providers: ["<<providersList.size()<<"]\n";
    for(auto it:providersList){
      std::cout <<"\t \t Task 0x"<<std::hex<< it.task
                <<", cl@0x"<< it.clAddr <<" dSize 0x"<<it.dSize <<"\n";
    }
    std::cout <<"\t Receivers: ["<<receiversList.size()<<"]\n";
    for(auto it:receiversList){
      std::cout <<"\t \t Task 0x"<<std::hex<< it.task
                <<", cl@0x"<< it.clAddr <<" dSize 0x"<<it.dSize <<"\n";
    }
#endif
    monitored_free(rawBuffer);
    return;
  }
