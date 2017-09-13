/**
 * @file gemm_blocked_cls.cc
 * @brief calling Stub for machsuite GEMM_BLOCKED functions.
 */
#include "gemm_blocked_cls.h"
#include <iostream>

/**
 * @brief emu-hmpsoc wrapper around machsuite gemm_blocked function.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1+iterOffset: TYPE_GEMMB[N_GEMMB] matrix 1
 * @params in_2+iterOffset: TYPE_GEMMB[N_GEMMB] matrix 2
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int gemm_blocked(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  TYPE_GEMMB *prod = (TYPE_GEMMB*) monitored_malloc(nb_iteration*sizeof(TYPE_GEMMB)*N_GEMMB);

#ifdef LOCAL_CPU_BUFFER
  TYPE_GEMMB m1[N_GEMMB], m2[N_GEMMB];
  TYPE_GEMMB loc_prod[N_GEMMB];
#else
  TYPE_GEMMB *m1, *m2;
  TYPE_GEMMB *loc_prod;
#endif

#ifdef MACHSUITE_DEBUG
  std::cout << "GEMM_BLOCKED started for "<< nb_iteration <<" iterations\n";
#endif
  for(uint i=0; i< nb_iteration; i++){
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)m1, (void*) ((uintptr_t)in+sizeof(uint32_t)
                               + i*(sizeof(TYPE_GEMMB)*(2*N_GEMMB))),
           sizeof(TYPE_GEMMB)*N_GEMMB);
    memcpy((void*)m2, (void*) ((uintptr_t)in+sizeof(uint32_t)+ sizeof(TYPE_GEMMB)*N_GEMMB
                               + i*(sizeof(TYPE_GEMMB)*(2*N_GEMMB))),
           sizeof(TYPE_GEMMB)*N_GEMMB);
#else
    m1 = (TYPE_GEMMB*)((uintptr_t)in+sizeof(uint32_t)
                 + i*(sizeof(TYPE_GEMMB)*(2*N_GEMMB)));
    m2 = (TYPE_GEMMB*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_GEMMB)*N_GEMMB
                 + i*(sizeof(TYPE_GEMMB)*(2*N_GEMMB)));
    loc_prod = (TYPE_GEMMB*)((uintptr_t)prod
                             + i*(sizeof(TYPE_GEMMB)*N_GEMMB));
#endif
    for (uint k=0; k<N_GEMMB; k++){ *((TYPE_GEMMB*)loc_prod +k)=0.0;}

    bbgemm(m1, m2, loc_prod);

#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)((uintptr_t)prod + i*(sizeof(TYPE_GEMMB)*N_GEMMB)), (void*)loc_prod, sizeof(TYPE_GEMMB)*N_GEMMB);
#endif
  }

#ifdef MACHSUITE_DEBUG
  std::cout << "GEMM_BLOCKED ended\n";
#endif
  // Output write in place in buf
  localAlloc = true;
  out = (TYPE_GEMMB*)(prod);
  return(0);
}

/**
 * @brief emu-hmpsoc wrapper around machsuite gemm_blocked_HW function.
 *        Allocate memory then call HW component through genericIp driver.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1+iterOffset: TYPE_GEMMB[N_GEMMB] matrix 1
 * @params in_2+iterOffset: TYPE_GEMMB[N_GEMMB] matrix 2
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int gemm_blocked_hw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  TYPE_GEMMB *dIn = (TYPE_GEMMB*)((uintptr_t)in+sizeof(uint32_t));
  TYPE_GEMMB *dOut = (TYPE_GEMMB*) monitored_malloc(nb_iteration*sizeof(TYPE_GEMMB)*N_GEMMB);


  // get Hardware address of input data
  uintptr_t dIn_hwAddr = monitored_VtoP(dIn);
  uintptr_t dOut_hwAddr = monitored_VtoP(dOut);
#ifdef MACHSUITE_DEBUG
  std::cout << "GEMM_BLOCKED_HW started for "<< nb_iteration <<" iterations\n"
            << "dIn@ 0x"<<std::hex<<dIn_hwAddr<<"\n"
            << "dOut@ 0x"<<std::hex<<dOut_hwAddr<<"\n";
#endif
  //Open device and init
  int dev_fd = open(DEV_GEMM_BLOCKED, O_RDWR);
  uint32_t args[3] = {nb_iteration, dIn_hwAddr, dOut_hwAddr};
  if(0>ioctl(dev_fd, IP_INIT, args)){close(dev_fd); return -1;}
  //start computation
  if(0>ioctl(dev_fd, IP_START, NULL)){close(dev_fd); return -1;}
  //close device
  close(dev_fd);

#ifdef MACHSUITE_DEBUG
  std::cout << "GEMM_BLOCKED_HW ended\n";
#endif
  // Output write in place in buf
  localAlloc = true;
  out = (TYPE_GEMMB*) dOut;
}

/**
 * @brief allocate input & output memory for gemm_blocked.
 *        generate a benchmark coherent set of random input data.
 *
 * @param iter: (uint32_t) number of iteration.
 * @param in: ptr to data input (should be set as NULL at function call)
 * @param out: ptr to data output (should be set as NULL at function call)
 */
void gemm_blocked_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut){
  //allocate memory
  lIn = sizeof(uint32_t) + iter * (sizeof(TYPE_GEMMB)*(2*N_GEMMB));
  lOut = iter * (sizeof(TYPE_GEMMB) *N_GEMMB);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  void* curData = in;

  //generate benchmark coherent random data
  *((uint32_t*) curData) = iter;
  curData = ((uint32_t*) curData) + 1;

  for(uint i=0; i< iter; i++){
    for(uint s=0; s< N_GEMMB; s++){ //double between 0 and 1
      *(((TYPE_GEMMB*)curData)+s)= (TYPE_GEMMB)((uint64_t)((TYPE_GEMMB)random()/(TYPE_GEMMB)(RAND_MAX)) & 0x7fffffffffffffff);
    }
    curData = ((TYPE_GEMMB*) curData) + (N_GEMMB);
    for(uint s=0; s< N_GEMMB; s++){
      *(((TYPE_GEMMB*)curData)+s)= (TYPE_GEMMB)((uint64_t)((TYPE_GEMMB)random()/(TYPE_GEMMB)(RAND_MAX)) & 0x7fffffffffffffff);
    }
    curData = ((TYPE_GEMMB*) curData) + (N_GEMMB);
  }
}

/**
 * @brief allocate input & output memory for gemm_blocked.
 *        generate golden inputs/outputs tests samples
 *
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params val: uninitialized ptr to golden outputs
 */
void gemm_blocked_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid){
  //samples array
  #include"gemm_blocked.samples"
  //allocate memory
  lIn = sizeof(uint32_t) + (sizeof(TYPE_GEMMB)*(2*N_GEMMB));
  lOut = (sizeof(TYPE_GEMMB) *N_GEMMB);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  valid = (void*) monitored_malloc(lOut);

  //Format inputs samples and iter => in array
  void* curData = in;

  //generate benchmark coherent random data
  *((uint32_t*) curData) = 1;
  curData = ((uint32_t*) curData) + 1;

    for(uint s=0; s< N_GEMMB; s++){ //double between 0 and 1
      *(((TYPE_GEMMB*)curData)+s)= gemmB_samples_mtx1[s];
    }
    curData = ((TYPE_GEMMB*) curData) + (N_GEMMB);
    for(uint s=0; s< N_GEMMB; s++){
      *(((TYPE_GEMMB*)curData)+s)= gemmB_samples_mtx2[s];
    }
  //Format outputs samples => valid array
    for(uint s=0; s< N_GEMMB; s++){
      *(((TYPE_GEMMB*)valid)+s)= gemmB_samples_outputs[s];
    }
}

/**
 * @brief return inputs and output dataSize
 *
 * @params iter: number of iteration
 * @params lIn: length of the generate inputs
 * @params lOut: length of the generate outputs buffers
 */
void gemm_blocked_getDataSize(uint iter, size_t &lIn, size_t &lOut){
  lIn = sizeof(uint32_t) + iter * (sizeof(TYPE_GEMMB)*(2*N_GEMMB));
  lOut = iter * (sizeof(TYPE_GEMMB) *N_GEMMB);
}
