/*
 * This test application is to validate the allocation mechanism in monitored memory for user process.
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>

#include "clMgmtAPI.h" //include all API stack containing power reading facilities and monitored_malloc facilities.

void usage(void)
{
	printf("*argv[0] -n <NUMBER_OF_ITERATION>i -i <NUMBER_OF_ALLOCATION> -s <SIZE> -p\n");
	printf("\t p: enable power printing @ each step\n");
	return;
}

int main(int argc, char *argv[])
{
	int c;
  uint nbIter=1, sizeAlloc=4, nbAlloc=1;
  uint printPower=0;
	/* Parse command line arguements */
	while((c = getopt(argc, argv, "n:i:s:ph")) != -1) {
		switch(c) {
		case 'n':
      nbIter =strtoul(optarg,NULL, 0);
			break;
		case 'i':
      nbAlloc =strtoul(optarg,NULL, 0);
			break;
		case 's':
      sizeAlloc =strtoul(optarg,NULL, 0);
			break;
    case 'p':
			printPower=1;
			break;
    case 'h':
			usage();
			return 0;
		default:
			printf("invalid option: %c\n", (char)c);
			usage();
			return -1;
		}
	}

  //init Random generator
  srandom(time(NULL));
  //Reset power values
  clmgmt::clMBase::resetCPM();

  uint n;
  for(n=0; n<nbIter; n++){
    char *mem[nbAlloc];
    printf("Iteration number %d [mem %lx]\n", n, (long)mem);
    uint a;
    for(a=0; a<nbAlloc;a++){
      //allocate memory;
      mem[a] = (char*) monitored_malloc(sizeAlloc*sizeof(char));
      if (NULL == mem[a]){
        printf("malloc error \n");
      }else{
        //write random value into them
        uint s;
        printf("write: %d bytes\n", sizeAlloc);
        for(s=0; s< sizeAlloc; s++){
          *(mem[a]+s) = rand();
        }
      }
    }

    for(a=0; a<nbAlloc; a++){
      if(NULL != mem[a]){
        monitored_free(mem[a]);
      }
    }

    if(1 == printPower){ // Print powerValue
      float tmpEng, tmpTime;
      clmgmt::clMBase::getCPM(tmpEng, tmpTime);
      std::cout << "POWER MONITORING COUNTER VALUES ------------------------\n";
      std::cout << "Cluster PM value: [Energy "<<tmpEng<<"; Time "<<tmpTime <<"]\n";
      std::cout << "----------------------------------------------------------------\n";

    }
  }

  return 0;
}
