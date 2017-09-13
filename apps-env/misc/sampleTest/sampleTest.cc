/*
 * Copyright (c) 2017 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: sampleTest.cc
 * machSuite sampleTest debug utility. Could be compile and test on the host (x86_arch).
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>

#include "machSuite_cls.h"

void usage(char * name)
{
  printf("%s:\n", name);
  printf("\t -t task:  task id\n");
  printf("\t -h:  used HW task\n");
  return;
}

int main(int argc, char ** argv)
{
  int c;
  uint task=1;
  uint hw=0;
  while((c = getopt(argc, argv, "t:h")) != -1) {
    switch(c) {
    case 't':
      task = strtoul(optarg,NULL, 0);
      break;
    case 'h':
      hw =1;
      break;
    default:
      printf("invalid option: %c\n", (char)c);
      usage(argv[0]);
      return EXIT_FAILURE;
    }
  }

  std::cout << "Start Test Case "<< task <<"-----------------------\n";
  //Setup task_id and data buffer
  uint task_id;
  if( 1==hw){task_id = (task << 24)| 0x40000000;}
  else{task_id = (task << 24);}

  void *dIn, *dOut, *dVal;
  size_t len_dataIn, len_dataOut;
  bool outAlloc=false;

  // Get sample inputs/outputs
  (machSuiteGetSamples.at(task << 24))(dIn, len_dataIn, dOut, len_dataOut, dVal);
  monitored_free(dOut); // no preallocation needed in this test case

  // Compute value from samples
  (machSuiteWorker.at(task_id))(dIn, dOut, outAlloc);


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

  monitored_free(dVal);
  if(outAlloc)
    monitored_free(dOut);
  monitored_free(dIn);
  std::cout << "End Test Case -----------------------\n";
  _exit(EXIT_SUCCESS);
}
