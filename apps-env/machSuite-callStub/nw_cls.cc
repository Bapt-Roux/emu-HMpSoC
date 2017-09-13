/**
 * @file nw_cls.cc
 * @brief calling Stub for machsuite NW functions.
 */
#include "nw_cls.h"
#include <iostream>

/**
 * @brief emu-hmpsoc wrapper around machsuite nw function.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1+iterOffset: char[ALEN] sequence A
 * @params in_2+iterOffset: char[BLEN] sequence B
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int nw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  char *alignSeq = (char*) monitored_malloc(nb_iteration*sizeof(char)*(ALEN+BLEN)*2);
  int M[(ALEN+1)*(BLEN+1)];
  char ptr[(ALEN+1)*(BLEN+1)];

#ifdef LOCAL_CPU_BUFFER
  char seqA[ALEN], seqB[BLEN];
  char alignA[ALEN+BLEN], alignB[ALEN+BLEN];
#else
  char *seqA, *seqB;
  char *alignA, *alignB;
#endif

#ifdef MACHSUITE_DEBUG
  std::cout << "NW started for "<< nb_iteration <<" iterations\n";
#endif
  for(uint i=0; i< nb_iteration; i++){
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)seqA, (void*)((uintptr_t)in+sizeof(uint32_t)+ i*(sizeof(char)*(ALEN+BLEN))), ALEN*sizeof(char));
    memcpy((void*)seqB, (void*)((uintptr_t)in+sizeof(uint32_t) + sizeof(char)*ALEN + i*(sizeof(char)*(ALEN+BLEN))), BLEN*sizeof(char));
#else
    seqA = (char*)((uintptr_t)in+sizeof(uint32_t)
                   + i*(sizeof(char)*(ALEN+BLEN)));
    seqB = (char*)((uintptr_t)in+sizeof(uint32_t) + sizeof(char)*ALEN
                   + i*(sizeof(char)*(ALEN+BLEN)));
    alignA = (char*)((uintptr_t)alignSeq
                     + i*(sizeof(char)*(ALEN+BLEN)*2));
    alignB = (char*)((uintptr_t)alignSeq + sizeof(char)*(ALEN+BLEN)
                     + i*(sizeof(char)*(ALEN+BLEN)*2));
#endif

    needwun(seqA, seqB, alignA, alignB,
                 M, ptr);
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)((uintptr_t)alignSeq + i*(sizeof(char)*(ALEN+BLEN)*2)),(void*) alignA, sizeof(char)*(ALEN+BLEN));

    memcpy((void*)((uintptr_t)alignSeq + sizeof(char)*(ALEN+BLEN)+ i*(sizeof(char)*(ALEN+BLEN)*2)),(void*) alignB, sizeof(char)*(ALEN+BLEN));

#endif
  }
#ifdef MACHSUITE_DEBUG
  std::cout << "NW ended\n";
#endif
  // Output write in place in buf
  localAlloc = true;
  out = (char*)(alignSeq);
  return(0);
}

/**
 * @brief emu-hmpsoc wrapper around machsuite nw_HW function.
 *        Allocate memory then call HW component through genericIp driver.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1+iterOffset: char[ALEN] sequence A
 * @params in_2+iterOffset: char[BLEN] sequence B
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int nw_hw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  char *dIn = (char*)((uintptr_t)in+sizeof(uint32_t));
  char *dOut = (char*) monitored_malloc(sizeof(char)*(ALEN+BLEN)*2);

  // get Hardware address of input data
  uintptr_t dIn_hwAddr = monitored_VtoP(dIn);
  uintptr_t dOut_hwAddr = monitored_VtoP(dOut);
#ifdef MACHSUITE_DEBUG
  std::cout << "NW_HW started for "<< nb_iteration <<" iterations\n"
            << "dIn@ 0x"<<std::hex<<dIn_hwAddr<<"\n"
            << "dInOut@ 0x"<<std::hex<<dOut_hwAddr<<"\n";
#endif
  //Open device and init
  int dev_fd = open(DEV_NW, O_RDWR);
  uint32_t args[3] = {nb_iteration, dIn_hwAddr, dOut_hwAddr};
  if(0>ioctl(dev_fd, IP_INIT, args)){close(dev_fd); return -1;}
  //start computation
  if(0>ioctl(dev_fd, IP_START, NULL)){close(dev_fd); return -1;}
  //close device
  close(dev_fd);

#ifdef MACHSUITE_DEBUG
  std::cout << "NW_HW ended\n";
#endif
  // Output write in place in buf
  localAlloc = true;
  out = (char*) dOut;
}

/**
 * @brief allocate input & output memory for nw.
 *        generate a benchmark coherent set of random input data.
 *
 * @param iter: (uint32_t) number of iteration.
 * @param in: ptr to data input (should be set as NULL at function call)
 * @param out: ptr to data output (should be set as NULL at function call)
 */
void nw_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut){
  //allocate memory
  lIn = sizeof(uint32_t) + iter * (sizeof(char)*(ALEN+BLEN));
  lOut = iter * (sizeof(char)*(ALEN+BLEN)*2);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  void* curData = in;

  //generate benchmark coherent random data
  *((uint32_t*) curData) = iter;
  curData = ((uint32_t*) curData) + 1;

  for(uint i=0; i< iter; i++){
    for(uint s=0; s< (ALEN); s++){ // seqA
      *(((char*)curData)+s)= (char)((random()%26)+'A');
    }
    curData = ((char*) curData) + (ALEN);
    for(uint s=0; s< (BLEN); s++){ // seqB
      *(((char*)curData)+s)= (char)((random()%26)+'A');
    }
    curData = ((char*) curData) + (BLEN);
  }
}

/**
 * @brief allocate input & output memory for nw.
 *        generate golden inputs/outputs tests samples
 *
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params val: uninitialized ptr to golden outputs
 */
void nw_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid){
  //samples array
  #include"nw.samples"

  //allocate memory
  lIn = sizeof(uint32_t) + (sizeof(char)*(ALEN+BLEN));
  lOut = (sizeof(char)*(ALEN+BLEN)*2);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  valid = (void*) monitored_malloc(lOut);

  //Format inputs samples and iter => in array
  void* curData = in;

  *((uint32_t*) curData) = 1;
  curData = ((uint32_t*) curData) + 1;

    for(uint s=0; s< (ALEN); s++){ // seqA
      *(((char*)curData)+s)= nw_samples_seqA[s];
    }
    curData = ((char*) curData) + (ALEN);
    for(uint s=0; s< (BLEN); s++){ // seqB
      *(((char*)curData)+s)= nw_samples_seqB[s];
    }
  //Format outputs samples => valid array
    curData = valid;
    for(uint s=0; s< (ALEN+BLEN); s++){ // seqA
      *(((char*)curData)+s)= nw_samples_outA[s];
    }
    curData = ((char*) curData) + (ALEN+BLEN);
    for(uint s=0; s< (ALEN+BLEN); s++){ // seqB
      *(((char*)curData)+s)= nw_samples_outB[s];
    }
}

/**
 * @brief return inputs and output dataSize
 *
 * @params iter: number of iteration
 * @params lIn: length of the generate inputs
 * @params lOut: length of the generate outputs buffers
 */
void nw_getDataSize(uint iter, size_t &lIn, size_t &lOut){
  lIn = sizeof(uint32_t) + iter * (sizeof(char)*(ALEN+BLEN));
  lOut = iter * (sizeof(char)*(ALEN+BLEN)*2);
}
