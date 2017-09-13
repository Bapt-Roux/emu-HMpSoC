/**
 * @file stenc2d_cls.cc
 * @brief calling Stub for machsuite STENC2D functions.
 */
#include "stenc2d_cls.h"
#include <iostream>

/**
 * @brief emu-hmpsoc wrapper around machsuite stenc2d function.
 *        NOTE: Data size are independents of the number of iterations.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1: TYPE_S2D[f_size] filter coeff
 * @params in_2: TYPE_S2D[ROW_S2D*COL_S2D] inputs
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int stenc2d(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
#ifdef LOCAL_CPU_BUFFER
  TYPE_S2D filter[f_size];
  TYPE_S2D inputs[ROW_S2D*COL_S2D];
#else
  TYPE_S2D *filter =(TYPE_S2D*)((uintptr_t)in+sizeof(uint32_t));
  TYPE_S2D *inputs =(TYPE_S2D*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_S2D)*f_size);
#endif

#ifdef MACHSUITE_DEBUG
  std::cout << "STENC2D started for "<< nb_iteration <<" iterations\n";
#endif

#ifdef LOCAL_CPU_BUFFER
  memcpy((void*)filter, (void*)((uintptr_t)in+sizeof(uint32_t)), sizeof(TYPE_S2D)*f_size);
  memcpy((void*)inputs, (void*)((uintptr_t)in+sizeof(uint32_t)+ sizeof(TYPE_S2D)*f_size),(ROW_S2D*COL_S2D)*sizeof(TYPE_S2D));
#endif
  for(uint i=0; i< nb_iteration; i++){
    stencil( inputs, inputs, filter); //outputs write in place
  }
#ifdef LOCAL_CPU_BUFFER
  memcpy((void*)((uintptr_t)in+sizeof(uint32_t)+ sizeof(TYPE_S2D)*f_size),(void*)inputs, (ROW_S2D*COL_S2D)*sizeof(TYPE_S2D));
#endif

#ifdef MACHSUITE_DEBUG
  std::cout << "STENC2D ended\n";
#endif
  // Output write in place in buf
  localAlloc = false;
  out = (TYPE_S2D*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_S2D)*f_size);
  return(0);
}

/**
 * @brief emu-hmpsoc wrapper around machsuite stenc2d_HW function.
 *        Allocate memory then call HW component through genericIp driver.
 *        NOTE: Data size are independents of the number of iterations.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1: TYPE_S2D[f_size] filter coeff
 * @params in_2: TYPE_S2D[ROW_S2D*COL_S2D] inputs
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int stenc2d_hw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  TYPE_S2D *dIn =(TYPE_S2D*)((uintptr_t)in+sizeof(uint32_t));
  TYPE_S2D *dInOut =(TYPE_S2D*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_S2D)*f_size);

  // get Hardware address of input data
  uintptr_t dIn_hwAddr = monitored_VtoP(dIn);
  uintptr_t dInOut_hwAddr = monitored_VtoP(dInOut);
#ifdef MACHSUITE_DEBUG
  std::cout << "STENC2D_HW started for "<< nb_iteration <<" iterations\n"
            << "dIn@ 0x"<<std::hex<<dIn_hwAddr<<"\n"
            << "dInOut@ 0x"<<std::hex<<dInOut_hwAddr<<"\n";
#endif
  //Open device and init
  int dev_fd = open(DEV_STENC2D, O_RDWR);
  uint32_t args[3] = {nb_iteration, dIn_hwAddr, dInOut_hwAddr};
  if(0>ioctl(dev_fd, IP_INIT, args)){close(dev_fd); return -1;}
  //start computation
  if(0>ioctl(dev_fd, IP_START, NULL)){close(dev_fd); return -1;}
  //close device
  close(dev_fd);

#ifdef MACHSUITE_DEBUG
  std::cout << "STENC2D_HW ended\n";
#endif
  // Output write in place in buf
  localAlloc = false;
  out = (TYPE_S2D*) dInOut;
}

/**
 * @brief allocate input & output memory for stenc2d.
 *        generate a benchmark coherent set of random input data.
 *
 * @param iter: (uint32_t) number of iteration.
 * @param in: ptr to data input (should be set as NULL at function call)
 * @param out: ptr to data output (should be set as NULL at function call)
 */
void stenc2d_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut){
  //allocate memory
  lIn = sizeof(uint32_t) + (sizeof(TYPE_S2D)*(ROW_S2D*COL_S2D +f_size));
  lOut = sizeof(TYPE_S2D)*(ROW_S2D*COL_S2D);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  void* curData = in;

  //generate benchmark coherent random data
  *((uint32_t*) curData) = iter;
  curData = ((uint32_t*) curData) + 1;

  for(uint s=0; s< (f_size); s++){ //filter
      *(((TYPE_S2D*)curData)+s+1)= (TYPE_S2D)random()&0x3ff;
    }
    curData = ((double*) curData) + (f_size);
    for(uint s=0; s< (ROW_S2D*COL_S2D); s++){ //inputs
      *(((TYPE_S2D*)curData)+s+1)= (TYPE_S2D)random()&0x3ff;
    }
    curData = ((TYPE_S2D*) curData) + (ROW_S2D*COL_S2D);
}

/**
 * @brief allocate input & output memory for stenc2d.
 *        generate golden inputs/outputs tests samples
 *
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params val: uninitialized ptr to golden outputs
 */
void stenc2d_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid){
  //samples array
  #include"stenc2d.samples"

  //allocate memory
  lIn = sizeof(uint32_t) + (sizeof(TYPE_S2D)*(ROW_S2D*COL_S2D +f_size));
  lOut = sizeof(TYPE_S2D)*(ROW_S2D*COL_S2D);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  valid = (void*) monitored_malloc(lOut);

  //Format inputs samples and iter => in array
  void* curData=in;
  *((uint32_t*) curData) = 1;
  curData = ((uint32_t*) curData) + 1;

  for(uint s=0; s< (f_size); s++){ //filter
    *(((TYPE_S2D*)curData)+s)= stenc2d_samples_filter[s];
  }
  curData = ((TYPE_S2D*) curData) + (f_size);
  for(uint s=0; s< (ROW_S2D*COL_S2D); s++){ //inputs
    *(((TYPE_S2D*)curData)+s)= stenc2d_samples_in[s];
  }
  //Format outputs samples => valid array
  for(uint s=0; s< (ROW_S2D*COL_S2D); s++){ //inputs
    *(((TYPE_S2D*)valid)+s)= stenc2d_samples_outputs[s];
  }
}

/**
 * @brief return inputs and output dataSize
 *
 * @params iter: number of iteration
 * @params lIn: length of the generate inputs
 * @params lOut: length of the generate outputs buffers
 */
void stenc2d_getDataSize(uint iter, size_t &lIn, size_t &lOut){
  lIn = sizeof(uint32_t) + (sizeof(TYPE_S2D)*(ROW_S2D*COL_S2D +f_size));
  lOut = sizeof(TYPE_S2D)*(ROW_S2D*COL_S2D);
}
