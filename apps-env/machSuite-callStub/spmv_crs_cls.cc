/**
 * @file spmv_crs_cls.cc
 * @brief calling Stub for machsuite SPMV_CRS functions.
 */
#include "spmv_crs_cls.h"
#include <iostream>

/**
 * @brief emu-hmpsoc wrapper around machsuite spmv_crs function.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1+iterOffset: TYPE_SPMVC[NNZ_SPMVC] the nonzeros of the matrix
 * @params in_2+iterOffset: int32_t[NNZ_SPMVC] the column index of the nonzeros
 * @params in_3+iterOffset: int32_t[N_SPMVC+1] the start of each row of nonzeros
 * @params in_4+iterOffset: TYPE_SPMVC[N_SPMVC]: the dense vector
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int spmv_crs(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  TYPE_SPMVC *dOut = (TYPE_SPMVC*) monitored_malloc(nb_iteration*sizeof(TYPE_SPMVC)*N_SPMVC);
#ifdef LOCAL_CPU_BUFFER
  TYPE_SPMVC val[NNZ_SPMVC], vec[N_SPMVC];
  int32_t cols[NNZ_SPMVC], rowDelimiters[N_SPMVC+1];
  TYPE_SPMVC loc_out[N_SPMVC];
#else
  TYPE_SPMVC *val, *vec;
  int32_t *cols, *rowDelimiters;
  TYPE_SPMVC *loc_out;
#endif

#ifdef MACHSUITE_DEBUG
  std::cout << "SPMV_CRS started for "<< nb_iteration <<" iterations\n";
#endif
  for(uint i=0; i< nb_iteration; i++){
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)val, (void*)((uintptr_t)in+sizeof(uint32_t) + i*(sizeof(TYPE_SPMVC)*(NNZ_SPMVC+ N_SPMVC) + sizeof(int32_t)*(NNZ_SPMVC+N_SPMVC+1))),
           sizeof(TYPE_SPMVC)*NNZ_SPMVC);
    memcpy((void*)cols, (void*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_SPMVC)*NNZ_SPMVC + i*(sizeof(TYPE_SPMVC)*(NNZ_SPMVC+ N_SPMVC) + sizeof(int32_t)*(NNZ_SPMVC+N_SPMVC+1))),
           sizeof(int32_t)*NNZ_SPMVC);
    memcpy((void*)rowDelimiters, (void*)((uintptr_t)in+sizeof(uint32_t) + (sizeof(TYPE_SPMVC)+sizeof(int32_t))*NNZ_SPMVC + i*(sizeof(TYPE_SPMVC)*(NNZ_SPMVC+ N_SPMVC) + sizeof(int32_t)*(NNZ_SPMVC+N_SPMVC+1))),
           sizeof(int32_t)*(N_SPMVC+1));
    memcpy((void*)vec, (void*)((uintptr_t)in+sizeof(uint32_t) + (sizeof(TYPE_SPMVC)+sizeof(int32_t))*(NNZ_SPMVC) + sizeof(int32_t)*(N_SPMVC+1)
                                      + i*(sizeof(TYPE_SPMVC)*(NNZ_SPMVC+ N_SPMVC) + sizeof(int32_t)*(NNZ_SPMVC+N_SPMVC+1))),
           sizeof(TYPE_SPMVC)*N_SPMVC);
#else
    val = (TYPE_SPMVC*)((uintptr_t)in+sizeof(uint32_t)
                        + i*(sizeof(TYPE_SPMVC)*(NNZ_SPMVC+ N_SPMVC) + sizeof(int32_t)*(NNZ_SPMVC+N_SPMVC+1)));
    cols = (int32_t*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_SPMVC)*NNZ_SPMVC
                      + i*(sizeof(TYPE_SPMVC)*(NNZ_SPMVC+ N_SPMVC) + sizeof(int32_t)*(NNZ_SPMVC+N_SPMVC+1)));
    rowDelimiters = (int32_t*)((uintptr_t)in+sizeof(uint32_t) + (sizeof(TYPE_SPMVC)+sizeof(int32_t))*NNZ_SPMVC
                               + i*(sizeof(TYPE_SPMVC)*(NNZ_SPMVC+ N_SPMVC) + sizeof(int32_t)*(NNZ_SPMVC+N_SPMVC+1)));
    vec = (TYPE_SPMVC*)((uintptr_t)in+sizeof(uint32_t)
                        + (sizeof(TYPE_SPMVC)+sizeof(int32_t))*(NNZ_SPMVC) + sizeof(int32_t)*(N_SPMVC+1)
                        + i*(sizeof(TYPE_SPMVC)*(NNZ_SPMVC+ N_SPMVC) + sizeof(int32_t)*(NNZ_SPMVC+N_SPMVC+1)));

    loc_out = (TYPE_SPMVC*)((uintptr_t)dOut
                        + i*(sizeof(TYPE_SPMVC)*(N_SPMVC)));
#endif
    spmv(val, cols, rowDelimiters, vec, loc_out);
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)((uintptr_t)dOut + i*(sizeof(TYPE_SPMVC)*(N_SPMVC))), (void*)loc_out,sizeof(TYPE_SPMVC)*N_SPMVC);
#endif
  }
#ifdef MACHSUITE_DEBUG
  std::cout << "SPMV_CRS ended\n";
#endif
  // Output write in place in buf
  localAlloc = true;
  out = (TYPE_SPMVC*)(dOut);
  return(0);
}

/**
 * @brief emu-hmpsoc wrapper around machsuite spmv_crs_HW function.
 *        Allocate memory then call HW component through genericIp driver.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1+iterOffset: TYPE_SPMVC[NNZ_SPMVC] the nonzeros of the matrix
 * @params in_2+iterOffset: int32_t[NNZ_SPMVC] the column index of the nonzeros
 * @params in_3+iterOffset: int32_t[N_SPMVC+1] the start of each row of nonzeros
 * @params in_4+iterOffset: TYPE_SPMVC[N_SPMVC]: the dense vector
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int spmv_crs_hw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  void *dIn = (void*)((uintptr_t)in+sizeof(uint32_t));
  TYPE_SPMVC *dOut = (TYPE_SPMVC*) monitored_malloc(nb_iteration*sizeof(TYPE_SPMVC)*N_SPMVC);


  // get Hardware address of input data
  uintptr_t dIn_hwAddr = monitored_VtoP(dIn);
  uintptr_t dOut_hwAddr = monitored_VtoP(dOut);
#ifdef MACHSUITE_DEBUG
  std::cout << "SPMV_CRS_HW started for "<< nb_iteration <<" iterations\n"
            << "dIn@ 0x"<<std::hex<<dIn_hwAddr<<"\n"
            << "dOut@ 0x"<<std::hex<<dOut_hwAddr<<"\n";
#endif
  //Open device and init
  int dev_fd = open(DEV_SPMV_CRS, O_RDWR);
  uint32_t args[3] = {nb_iteration, dIn_hwAddr, dOut_hwAddr};
  if(0>ioctl(dev_fd, IP_INIT, args)){close(dev_fd); return -1;}
  //start computation
  if(0>ioctl(dev_fd, IP_START, NULL)){close(dev_fd); return -1;}
  //close device
  close(dev_fd);

#ifdef MACHSUITE_DEBUG
  std::cout << "SPMV_CRS_HW ended\n";
#endif
  // Output write in place in buf
  localAlloc = true;
  out = (TYPE_SPMVC*) dOut;
}

/**
 * @brief allocate input & output memory for spmv_crs.
 *        generate a benchmark coherent set of random input data.
 *
 * @param iter: (uint32_t) number of iteration.
 * @param in: ptr to data input (should be set as NULL at function call)
 * @param out: ptr to data output (should be set as NULL at function call)
 */
void spmv_crs_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut){
  //allocate memory
  lIn = sizeof(uint32_t) + iter * (sizeof(TYPE_SPMVC)*(NNZ_SPMVC+ N_SPMVC) + sizeof(int32_t)*(NNZ_SPMVC+N_SPMVC+1));
  lOut = iter * (sizeof(TYPE_SPMVC) *N_SPMVC);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  void* curData = in;

  //generate benchmark coherent random data
  *((uint32_t*) curData) = iter;
  curData = ((uint32_t*) curData) + 1;

  for(uint i=0; i< iter; i++){
    for(uint s=0; s< NNZ_SPMVC; s++){ //vals: [range -1024; 1024]
      *(((TYPE_SPMVC*)curData)+s)= (TYPE_SPMVC)(random()/(TYPE_SPMVC)(RAND_MAX))*1024;
    }
    curData = ((TYPE_SPMVC*) curData) + (NNZ_SPMVC);
    for(uint s=0; s< NNZ_SPMVC; s++){ //cols
      *(((int32_t*)curData)+s)= (int32_t)((random()&0x7fffffff)%512);
    }
    curData = ((int32_t*) curData) + (NNZ_SPMVC);
    int32_t row=0;
    for(uint s=0; s< (N_SPMVC+1); s++){ //rowDelimiters
      row = (row +(random()%12))%N_SPMVC;
      *(((int32_t*)curData)+s)= row;
    }
    curData = ((int32_t*) curData) + (N_SPMVC+1);
    for(uint s=0; s< N_SPMVC; s++){ //vec: double range [0;1]
      *(((TYPE_SPMVC*)curData)+s)= (TYPE_SPMVC)((uint64_t)((TYPE_SPMVC)random()/(TYPE_SPMVC)(RAND_MAX))
                                                & 0x7fffffffffffffff);
    } curData = ((TYPE_SPMVC*) curData) + (N_SPMVC);}
}

/**
 * @brief allocate input & output memory for spmv_crs.
 *        generate golden inputs/outputs tests samples
 *
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params val: uninitialized ptr to golden outputs
 */
void spmv_crs_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid){
  //samples array
  #include"spmv_crs.samples"

  //allocate memory
  lIn = sizeof(uint32_t) + (sizeof(TYPE_SPMVC)*(NNZ_SPMVC+ N_SPMVC) + sizeof(int32_t)*(NNZ_SPMVC+N_SPMVC+1));
  lOut = (sizeof(TYPE_SPMVC) *N_SPMVC);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  valid = (void*) monitored_malloc(lOut);

  //Format inputs samples and iter => in array
  void* curData = in;

  *((uint32_t*) curData) = 1;
  curData = ((uint32_t*) curData) + 1;

    for(uint s=0; s< NNZ_SPMVC; s++){ //val
      *(((TYPE_SPMVC*)curData)+s)= spmvC_samples_val[s];
    }
    curData = ((TYPE_SPMVC*) curData) + (NNZ_SPMVC);
    for(uint s=0; s< NNZ_SPMVC; s++){ //cols
      *(((uint32_t*)curData)+s)= spmvC_samples_cols[s];
    }
    curData = ((int32_t*) curData) + (NNZ_SPMVC);
    for(uint s=0; s< (N_SPMVC+1); s++){ //rowDelimiters
      *(((int32_t*)curData)+s)= spmvC_samples_rowsD[s];
    }
    curData = ((int32_t*) curData) + (N_SPMVC+1);
    for(uint s=0; s< N_SPMVC; s++){ //vec
      *(((TYPE_SPMVC*)curData)+s)= spmvC_samples_vec[s];
    }
  //Format outputs samples => valid array
    for(uint s=0; s< N_SPMVC; s++){ //vec
      *(((TYPE_SPMVC*)valid)+s)= spmvC_samples_outputs[s];
    }
}

/**
 * @brief return inputs and output dataSize
 *
 * @params iter: number of iteration
 * @params lIn: length of the generate inputs
 * @params lOut: length of the generate outputs buffers
 */
void spmv_crs_getDataSize(uint iter, size_t &lIn, size_t &lOut){
  lIn = sizeof(uint32_t) + iter * (sizeof(TYPE_SPMVC)*(NNZ_SPMVC+ N_SPMVC) + sizeof(int32_t)*(NNZ_SPMVC+N_SPMVC+1));
  lOut = iter * (sizeof(TYPE_SPMVC) *N_SPMVC);
}
