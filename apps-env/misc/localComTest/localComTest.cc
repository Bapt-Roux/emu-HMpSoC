/*
 * Copyright (c) 2017 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: localComTest.c
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

void usage(char * name)
{
	printf("%s -n <NUMBER_OF_ITERATION> -p\n", name);
	printf("\t p: enable power printing @ each step\n");
	return;
}

int main(int argc, char ** argv)
{
	int c;
  uint nbIter=1;
  uint printPower=0;

	/* Parse command line arguments */
	while((c = getopt(argc, argv, "n:s:rph")) != -1) {
		switch(c) {
		case 'n':
      nbIter =strtoul(optarg,NULL, 0);
			break;
		// case 's':
    //   sizeAlloc =strtoul(optarg,NULL, 0);
		// 	break;
    case 'p':
			printPower=1;
			break;
    case 'h':
			usage(argv[0]);
			return 0;
		default:
			printf("invalid option: %c\n", (char)c);
			usage(argv[0]);
			return -1;
		}
	}


  //instantiate Master Manager
  clMaster mgmtMaster(NOC_DEVICE);

  uint n;
  uint task = MGMT_TASKID;
  //Memory size and ptr
  uint len_data=(16*sizeof(uint8_t));
  void *dIn, *dOut;

  for(n=0; n<nbIter; n++){
    // Setup providers & receivers
    std::deque<comProps> providers, receivers;
    comProps tmpRcv, tmpPrv;

    //Providers
    tmpPrv.task = task; //previous spawned task
    tmpPrv.dSize = len_data;
    tmpPrv.clAddr = (mgmtMaster.getNocApi().getClAddr());
    providers.push_back(tmpPrv);

    if( (nbIter-1)==n){ // Receivers is master
      tmpRcv.task = MGMT_TASKID;
    }else{
      tmpRcv.task = (ID_AES <<TASK_TO_TID)+ ((n+1)<<TASK_IDOFFSET);
    }
    tmpRcv.dSize = len_data;
    tmpRcv.clAddr = (mgmtMaster.getNocApi().getClAddr());
    receivers.push_back(tmpRcv);

    //spawn an AES slave worker locally
    task = (ID_AES <<TASK_TO_TID)+(n<<TASK_IDOFFSET);
    trgtId_t trgt = std::make_pair(task, (mgmtMaster.getNocApi().getClAddr()));
    mgmtMaster.askSlaveSubscribe(trgt);
    mgmtMaster.setProviderAndReceiver(trgt, providers, receivers);

    //send data if first iter
    if( 0 == n){ //alloc data and send it
      dIn = monitored_malloc((size_t) len_data);
      mgmtMaster.b_sendData(trgt, dIn, len_data);
    }
    //Receive data if last
    if((nbIter-1)==n){
      mgmtMaster.b_rcvData(trgt, dOut, len_data);
    }

    if(1 == printPower){ // Print powerValue
      float tmpEng, tmpTime;
      clmgmt::clMBase::getCPM(tmpEng, tmpTime);
      std::cout << "POWER MONITORING COUNTER VALUES ------------------------\n";
      std::cout << "Cluster PM value: [Energy "<<tmpEng<<"; Time "<<tmpTime <<"]\n";
      std::cout << "----------------------------------------------------------------\n";
    }
  }
  //Free memory for next iteration
  monitored_free(dIn);
  monitored_free(dOut);

  return(EXIT_SUCCESS);
}
