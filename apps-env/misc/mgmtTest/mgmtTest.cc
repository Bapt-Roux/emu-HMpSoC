/*
 * Copyright (c) 2017 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: mgmtTest.c
 * Test application for cluster management facilities
 */
#include "clMgmtAPI.h"
extern "C" {
#include "monAlloc.h"
}
#include "machSuite-callStub/aes_cls.h"
#define GUARD 10

using namespace napi;
using namespace clmgmt;
void startMaster(uint iter, uint clId, uint task, char hw, char sample){
  clMaster main("/dev/sc_noc");

  const std::map<uint,noc::clPos> confCl = main.getNocApi().getClusters();
  std::cout << "NoC configuration read from Master cluster is:\n";
  for( auto & it_NoC: confCl){
    std::cout << "\t @" <<std::hex<< it_NoC.first
              << " =>["<<(int)it_NoC.second.x_pos
              << ","<<(int)it_NoC.second.y_pos << "]\n";
  }

  std::cout << "Start task "<<task<<"_"<<((hw==1)?"HW":"SW")<<"\n"
            << "\t "<<iter << " iterations run on target clusterId 0x" <<std::hex<< clId<<std::endl;

  //Build task id and clAddr
  uint task_id;
  uint gen_id = task << 24;
  if( 1==hw){task_id = gen_id | 0x40000000;}
  else{task_id = gen_id;}
  trgtId_t trgt = std::make_pair(task_id, (clId<<28));

  //Local execution
  bool localExec = ((main.getNocApi().getClAddr() >> 28) == clId);

  //setup data buffer
  void *dIn, *dOut, *dRef, *dVal;
  size_t len_dataIn, len_dataOut;
  bool outAlloc=false;
  bool refAlloc=false;
  if(1== sample){ // get sample inputs/outputs
    std::cout << "Use sample data\n";
    (machSuiteGetSamples.at(gen_id& NOC_JOBMASK))(dIn, len_dataIn, dOut, len_dataOut, dVal);
    outAlloc = true;
  }else{
    std::cout << "Use random data\n";
    (machSuiteGenData.at(gen_id& NOC_JOBMASK))(iter, dIn, len_dataIn, dOut, len_dataOut);
    dRef = monitored_malloc(len_dataIn);
    refAlloc = true;
    for(uint c=0; c<len_dataIn; c++){((uint8_t*)dRef)[c] = ((uint8_t*)dIn)[c];}
    (machSuiteWorker.at(gen_id& NOC_JOBMASK))(dRef, dVal, outAlloc);
  }
  std::cout << "inputs/outputs data generated \n";
#ifdef DATA_DUMP_DEBUG
  std::cout << " Inputs data generated: \n";
  for(uint c=0; c<len_dataIn; c++)
    std::cout << "["<<std::dec <<c<<"] => "<< (uint) ((uint8_t*)dIn)[c] <<"\n";
#endif
  //ask for worker
  main.askSlaveSubscribe(trgt);

  // send data properties
  std::deque<comProps> providers, receivers;
  comProps tmp;
  tmp.clAddr = main.getNocApi().getClAddr();
  tmp.task = trgt.first;
  tmp.dSize = len_dataIn;
  providers.push_back(tmp);
  if (localExec){ //local master EP
    tmp.task = MGMT_TASKID;
  }else{
    tmp.task =trgt.first; //write back to master => same task_id
  }
  tmp.dSize = len_dataOut;
  receivers.push_back(tmp);

  main.setProviderAndReceiver(trgt, providers, receivers);
  main.b_sendData(trgt, dIn, len_dataIn);
  main.b_rcvData(trgt, dOut, len_dataOut);

  std::cout << "remote worker end\n";
#ifdef DATA_DUMP_DEBUG
  std::cout << " Outputs data received vs valid: \n";
  for(uint c=0; c<len_dataOut; c++)
  std::cout << "["<<std::dec <<c<<"] => "<< (uint) ((uint8_t*)dOut)[c] <<" == "<< (uint)((uint8_t*)dVal)[c]<<"\n";
#endif

  //Check results
  uint error=0;
  for(uint c=0; c<len_dataOut; c++)
    if(((uint8_t*)dOut)[c] != ((uint8_t*)dVal)[c]){
      error++;
    }
  std::cout << " Integer errors "<<std::dec <<error<<"\n";

  error=0;
  for(uint c=0; c<len_dataOut/sizeof(double); c++)
    if(fabs(((double*)dOut)[c] - ((double*)dVal)[c]) > (1.0e-6)){
      error++;
    }
  std::cout << " Float errors "<<std::dec <<error<<"\n";

  if(outAlloc)
    monitored_free(dVal);
  if(refAlloc)
    monitored_free(dRef);
  monitored_free(dIn);
  monitored_free(dOut);
  std::cout << "End Test Case -----------------------\n";
  if (!localExec){ main.getNocApi().unregisterTask(trgt.first);}
 _exit(EXIT_SUCCESS);
}

void startEndPoint(){
  clEndPoint ep("/dev/sc_noc");
}

void usage(char * name)
{
  printf("%s -M|-e:\n", name);
  printf("\t -M  master cluster\n");
  printf("\t\t -c addr:  target cluster addr\n");
  printf("\t\t -i  nb_iterations\n");
  printf("\t\t -t task:  task id\n");
  printf("\t\t -h  used hw impl\n");
  printf("\t -e  endpoint cluster\n");
  printf("default: endpoint mode\n");
  return;
}

int main(int argc, char ** argv)
{
  int c;
  char mode='e';
  //vars for master submode
  char hw=0;
  char sample=0;
  uint64_t trgtClId=0x8;
  uint task=0;
  uint iter=1;
  while((c = getopt(argc, argv, "Me")) != -1) {
    switch(c) {
    case 'M':
      mode = 'M';
      while((c = getopt(argc, argv, "c:t:i:hs")) != -1) {
        switch(c) {
        case 'c':
          trgtClId = strtoul(optarg,NULL, 0);
          break;
        case 'i':
          iter = strtoul(optarg,NULL, 0);
          break;
        case 't':
          task = strtoul(optarg,NULL, 0);
          break;
        case 'h': //hw task
          hw = 1;
          break;
        case 's': //use sample tests
          sample = 1;
          break;
        }
      }
      break;

    case 'e': //default case
      mode = 'e';
      break;

    default:
      printf("invalid option: %c\n", (char)c);
      usage(argv[0]);
      return EXIT_FAILURE;
    }
  }

  if('M' == mode){
    startMaster(iter, trgtClId, task, hw, sample);
    printf("master_return\n");
  }else{
    startEndPoint();
    printf("ENDPOINT_return\n");
  }
  return(EXIT_SUCCESS);
}
