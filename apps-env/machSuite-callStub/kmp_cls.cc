/**
 * @file kmp_cls.cc
 * @brief calling Stub for machsuite KMP functions.
 */
#include "kmp_cls.h"
#include <iostream>

/**
 * @brief emu-hmpsoc wrapper around machsuite kmp function.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1+iterOffset: char[PATTERN_SIZE] pattern
 * @params in_2+iterOffset: double[STRING_SIZE] input text
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int kmp(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  int32_t *matchs = (int32_t*) monitored_malloc(nb_iteration+sizeof(int32_t));
  int status =0;
#ifdef LOCAL_CPU_BUFFER
  char pattern[PATTERN_SIZE], text[STRING_SIZE];
  int32_t loc_matchs[1];
  int32_t kmpNext[PATTERN_SIZE];
#else
  char *pattern, *text;
  int32_t *loc_matchs;
  int32_t kmpNext[PATTERN_SIZE];
#endif

#ifdef MACHSUITE_DEBUG
  std::cout << "KMP started for "<< nb_iteration <<" iterations\n";
#endif
  for(uint i=0; i< nb_iteration; i++){
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)pattern, (void*)((uintptr_t)in+sizeof(uint32_t) + i*(sizeof(char)*(PATTERN_SIZE + STRING_SIZE))),
           sizeof(char)*PATTERN_SIZE);
    memcpy((void*)text, (void*)((uintptr_t)in+sizeof(uint32_t) +sizeof(char)*PATTERN_SIZE + i*(sizeof(char)*(PATTERN_SIZE + STRING_SIZE))),
           sizeof(char)*STRING_SIZE);
#else
    pattern = (char*)((uintptr_t)in+sizeof(uint32_t)
                      + i*(sizeof(char)*(PATTERN_SIZE + STRING_SIZE)));
    text = (char*)((uintptr_t)in+sizeof(uint32_t) +sizeof(char)*PATTERN_SIZE
                      + i*(sizeof(char)*(PATTERN_SIZE + STRING_SIZE)));
    loc_matchs = (int32_t*)((uintptr_t)matchs + i*sizeof(int32_t));
#endif
    status+= kmp(pattern, text, kmpNext, loc_matchs);
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)((uintptr_t)matchs + i*sizeof(int32_t)),(void*)loc_matchs,  sizeof(uint32_t));
#endif
  }
#ifdef MACHSUITE_DEBUG
  std::cout << "KMP ended\n";
#endif
  // Output write in place in buf
  localAlloc = true;
  out = (int32_t*)(matchs);
  return(status);
}

/**
 * @brief emu-hmpsoc wrapper around machsuite kmp_HW function.
 *        Allocate memory then call HW component through genericIp driver.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1+iterOffset: char[PATTERN_SIZE] pattern
 * @params in_2+iterOffset: double[STRING_SIZE] input text
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int kmp_hw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  char *dIn = (char*)((uintptr_t)in+sizeof(uint32_t));
  int32_t *dOut = (int32_t*) monitored_malloc(nb_iteration*sizeof(int32_t));

  // get Hardware address of input data
  uintptr_t dIn_hwAddr = monitored_VtoP(dIn);
  uintptr_t dOut_hwAddr = monitored_VtoP(dOut);
#ifdef MACHSUITE_DEBUG
  std::cout << "KMP_HW started for "<< nb_iteration <<" iterations\n"
            << "dIn@ 0x"<<std::hex<<dIn_hwAddr<<"\n"
            << "dOut@ 0x"<<std::hex<<dOut_hwAddr<<"\n";
#endif
  //Open device and init
  int dev_fd = open(DEV_KMP, O_RDWR);
  uint32_t args[3] = {nb_iteration, dIn_hwAddr, dOut_hwAddr};
  if(0>ioctl(dev_fd, IP_INIT, args)){close(dev_fd); return -1;}
  //start computation
  if(0>ioctl(dev_fd, IP_START, NULL)){close(dev_fd); return -1;}
  //close device
  close(dev_fd);

#ifdef MACHSUITE_DEBUG
  std::cout << "KMP_HW ended\n";
#endif
  // Output write in place in buf
  localAlloc = true;
  out = (int32_t*)(dOut);
}

/**
 * @brief allocate input & output memory for kmp.
 *        generate a benchmark coherent set of random input data.
 *
 * @param iter: (uint32_t) number of iteration.
 * @param in: ptr to data input (should be set as NULL at function call)
 * @param out: ptr to data output (should be set as NULL at function call)
 */
void kmp_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut){
  //allocate memory
  lIn = sizeof(uint32_t) + iter * (sizeof(char)*(PATTERN_SIZE + STRING_SIZE));
  lOut = iter * (sizeof(int32_t));

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  void* curData = in;

  //generate benchmark coherent random data
  *((uint32_t*) curData) = iter;
  curData = ((uint32_t*) curData) + 1;

  for(uint i=0; i< iter; i++){
    for(uint s=0; s< (PATTERN_SIZE-1); s++){ // pattern
      *(((char*)curData)+s)= (char)random();
    }
    *(((char*)curData)+(PATTERN_SIZE-1))= '\0';
    curData = ((char*) curData) + (PATTERN_SIZE);

    for(uint s=0; s< (STRING_SIZE-1); s++){ // text
      *(((char*)curData)+s)= (char)random();
    }
    *(((char*)curData)+(STRING_SIZE-1))= '\0';
    curData = ((char*) curData) + (STRING_SIZE);
  }
}

/**
 * @brief allocate input & output memory for kmp.
 *        generate golden inputs/outputs tests samples
 *
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params val: uninitialized ptr to golden outputs
 */
void kmp_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid){
  //samples array
  #include"kmp.samples"

  //allocate memory
  lIn = sizeof(uint32_t) + (sizeof(char)*(PATTERN_SIZE + STRING_SIZE));
  lOut = (sizeof(int32_t));

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  valid = (void*) monitored_malloc(lOut);

  //Format inputs samples and iter => in array
  void* curData = in;
  *((uint32_t*) curData) = 1;
  curData = ((uint32_t*) curData) + 1;

    for(uint s=0; s< (PATTERN_SIZE); s++){ // pattern
      *(((char*)curData)+s)= kmp_samples_pattern[s];
    }
    curData = ((char*) curData) + (PATTERN_SIZE);

    for(uint s=0; s< (STRING_SIZE); s++){ // text
      *(((char*)curData)+s)= kmp_samples_text[s];
    }
  //Format outputs samples => valid array
    *((int*)valid)= kmp_samples_outputs;
}

/**
 * @brief return inputs and output dataSize
 *
 * @params iter: number of iteration
 * @params lIn: length of the generate inputs
 * @params lOut: length of the generate outputs buffers
 */
void kmp_getDataSize(uint iter, size_t &lIn, size_t &lOut){
  lIn = sizeof(uint32_t) + iter * (sizeof(char)*(PATTERN_SIZE + STRING_SIZE));
  lOut = iter * (sizeof(int32_t));
}
