/*
 * Copyright (c) 2017 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */






/*
 * File: mgmtTest.cc
 * Run each dwarfs to get comIn and comOut
 */

#include<iostream>
extern "C" {
#include "monAlloc.h"
}
#include "clMgmtAPI.h"
using namespace clmgmt;

void usage(char * name)
{
	printf("%s -t <task_id> -c\n", name);
	printf("\t c: print input and output data size\n");
	return;
}

int main(int argc, char ** argv){
	int c;
  uint task = 0;
  bool getCom = false;
	/* Parse command line arguements */
	while((c = getopt(argc, argv, "ct:h")) != -1) {
		switch(c) {
		case 'c':
      getCom = true;
			break;
		case 't':
      task =strtoul(optarg,NULL, 0);
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

  if(getCom){
  size_t lIn0, lOut0;
  size_t lIn1, lOut1;
  for(auto it: machSuiteDataSize){
    it.second(0, lIn0, lOut0);
    it.second(1, lIn1, lOut1);
    std::cout << "Input and output communication size---------------------------------------- \n"
              << " machSuite task id: 0x" << std::hex << it.first
              << std::dec<<"\n \t input size => stat " << lIn0 << " dyn " << lIn1 - lIn0
              << std::dec<<"\n \t output size => stat " << lOut0 << " dyn " << lOut1 - lOut0
              << "\n--------------------------------------------------------------------------------\n";
  }
  } else{
    float tmpEng, tmpTime;

    void *dIn, *dOut, *dVal;
    size_t len_dataIn, len_dataOut;
    bool outAlloc=false;

    uint task_id = task << 24;
    std::cout << "Communication cost involved in computation ---------------------------------------- \n"
              << " machSuite task id: 0x" << std::hex << task_id << " SW: \n";
    clMBase::resetCPM();
    (machSuiteGetSamples.at((task<<24)& NOC_JOBMASK))(dIn, len_dataIn, dOut, len_dataOut, dVal);
    (machSuiteWorker.at(task_id & NOC_JOBMASK))(dIn, dOut, outAlloc);
    if(outAlloc){
      monitored_free(dVal);
      monitored_free(dOut);
    }
    monitored_free(dIn);
    clMBase::getCPM(tmpEng, tmpTime);
    std::cout << "Task 0x"<<std::hex<< task_id<<" PM value: [Energy "<<tmpEng<<"; Time "<<tmpTime <<"]\n";
    std::cout <<"************************************************************\n";

    task_id = (task<< 24)| 0x40000000;
    std::cout << " machSuite task id: 0x" << std::hex << task_id << " HW: \n";
    clMBase::resetCPM();
    (machSuiteGetSamples.at((task<<24)& NOC_JOBMASK))(dIn, len_dataIn, dOut, len_dataOut, dVal);
    (machSuiteWorker.at(task_id & NOC_JOBMASK))(dIn, dOut, outAlloc);
    if(outAlloc){
      monitored_free(dVal);
      monitored_free(dOut);
    }
    monitored_free(dIn);
    clMBase::getCPM(tmpEng, tmpTime);
    std::cout << "Task 0x"<<std::hex<< task_id<<" PM value: [Energy "<<tmpEng<<"; Time "<<tmpTime <<"]\n";
    std::cout <<"************************************************************\n";

  }

  return(EXIT_SUCCESS);
}
