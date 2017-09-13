/**
 * @file sort_merge_cls.cc
 * @brief calling Stub for machsuite SORT_MERGE functions.
 */
#include "sort_merge_cls.h"
#include <iostream>

/**
 * @brief emu-hmpsoc wrapper around machsuite sort_merge function.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1+iterOffset: TYPE_SM[SIZE] dataArray
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int sort_merge(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
#ifdef LOCAL_CPU_BUFFER
  TYPE_SM dIn[SIZE];
#else
  TYPE_SM *dIn;
#endif

#ifdef MACHSUITE_DEBUG
  std::cout << "SORT_MERGE started for "<< nb_iteration <<" iterations\n";
#endif
  for(uint i=0; i< nb_iteration; i++){
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)dIn, (void*)((uintptr_t)in+sizeof(uint32_t) + i*(sizeof(TYPE_SM)*SIZE)),sizeof(TYPE_SM)*SIZE);
#else
    dIn = (TYPE_SM*)((uintptr_t)in+sizeof(uint32_t)
                     + i*(sizeof(TYPE_SM)*SIZE));
#endif

    ms_mergesort(dIn);
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)((uintptr_t)in+sizeof(uint32_t) + i*(sizeof(TYPE_SM)*SIZE)), (void*)dIn, sizeof(TYPE_SM)*SIZE);
#endif
  }
#ifdef MACHSUITE_DEBUG
  std::cout << "SORT_MERGE ended\n";
#endif
  // Output write in place in buf
  localAlloc = false;
  out = (TYPE_SM*)((uintptr_t)in+sizeof(uint32_t));
  return(0);
}

/**
 * @brief emu-hmpsoc wrapper around machsuite sort_merge_HW function.
 *        Allocate memory then call HW component through genericIp driver.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1+iterOffset: TYPE_SM[SIZE] dataArray
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int sort_merge_hw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  double *dIn = (double*)((uintptr_t)in+sizeof(uint32_t));

  // get Hardware address of input data
  uintptr_t dIn_hwAddr = monitored_VtoP(dIn);
#ifdef MACHSUITE_DEBUG
  std::cout << "SORT_MERGE_HW started for "<< nb_iteration <<" iterations\n"
            << "dIn@ 0x"<<std::hex<<dIn_hwAddr<<"\n"
            << "dOut@ 0x"<<std::hex<<dIn_hwAddr<<"\n";
#endif
  //Open device and init
  int dev_fd = open(DEV_SORT_MERGE, O_RDWR);
  uint32_t args[3] = {nb_iteration, dIn_hwAddr, dIn_hwAddr};
  if(0>ioctl(dev_fd, IP_INIT, args)){close(dev_fd); return -1;}
  //start computation
  if(0>ioctl(dev_fd, IP_START, NULL)){close(dev_fd); return -1;}
  //close device
  close(dev_fd);

#ifdef MACHSUITE_DEBUG
  std::cout << "SORT_MERGE_HW ended\n";
#endif
  // Output write in place in buf
  localAlloc = false;
  out = (TYPE_SM*) dIn;
}

/**
 * @brief allocate input & output memory for sort_merge.
 *        generate a benchmark coherent set of random input data.
 *
 * @param iter: (uint32_t) number of iteration.
 * @param in: ptr to data input (should be set as NULL at function call)
 * @param out: ptr to data output (should be set as NULL at function call)
 */
void sort_merge_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut){
  //allocate memory
  lIn = sizeof(uint32_t) + iter * (sizeof(TYPE_SM)*SIZE);
  lOut = iter * (sizeof(TYPE_SM)*SIZE);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  void* curData = in;

  //generate benchmark coherent random data
  *((uint32_t*) curData) = iter;
  curData = ((uint32_t*) curData) + 1;

  for(uint i=0; i< iter; i++){
    for(uint s=0; s< (SIZE); s++){ //positive int32_t
      *(((TYPE_SM*)curData)+s)= (TYPE_SM)(random()& 0x7fffffff);
    }
    curData = ((double*) curData) + (SIZE);
  }
}

/**
 * @brief allocate input & output memory for sort_merge.
 *        generate golden inputs/outputs tests samples
 *
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params val: uninitialized ptr to golden outputs
 */
void sort_merge_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid){
  //samples array
#include"sort_merge.samples"

  //allocate memory
  lIn = sizeof(uint32_t) + (sizeof(TYPE_SM)*SIZE);
  lOut = (sizeof(TYPE_SM)*SIZE);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  valid = (void*) monitored_malloc(lOut);

  //Format inputs samples and iter => in array
  void* curData = in;
  *((uint32_t*) curData) = 1;
  curData = ((uint32_t*) curData) + 1;

    for(uint s=0; s< (SIZE); s++){
      *(((TYPE_SM*)curData)+s)= sortM_samples_in[s];
    }
  //Format outputs samples => valid array
    for(uint s=0; s< (SIZE); s++){
      *(((TYPE_SM*)valid)+s)= sortM_samples_outputs[s];
    }
}

/**
 * @brief return inputs and output dataSize
 *
 * @params iter: number of iteration
 * @params lIn: length of the generate inputs
 * @params lOut: length of the generate outputs buffers
 */
void sort_merge_getDataSize(uint iter, size_t &lIn, size_t &lOut){
  lIn = sizeof(uint32_t) + iter * (sizeof(TYPE_SM)*SIZE);
  lOut = iter * (sizeof(TYPE_SM)*SIZE);
}
