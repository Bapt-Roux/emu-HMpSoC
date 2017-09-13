/**
 * @file fft_strided_cls.cc
 * @brief calling Stub for machsuite FFT_STRIDED functions.
 */
#include "fft_strided_cls.h"
#include <iostream>

/**
 * @brief emu-hmpsoc wrapper around machsuite fft_strided function.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_00+iterOffsetIN: double[FFT_SIZE/2] twiddle factor (real)
 * @params in_01+iterOffsetIN: double[FFT_SIZE/2] twiddle factor (img)
 * @params in_10+iterOffsetOUT: double[FFT_SIZE] signal (real)
 * @params in_11+iterOffsetOUT: double[FFT_SIZE] signal (img)
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int fft_strided(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
#ifdef LOCAL_CPU_BUFFER
  double dReal[FFT_SIZE], dImg[FFT_SIZE];
  double twReal[FFT_SIZE/2], twImg[FFT_SIZE/2];
#else
  double *dReal, *dImg;
  double *twReal, *twImg;
#endif

#ifdef MACHSUITE_DEBUG
  std::cout << "FFT_STRIDED started for "<< nb_iteration <<" iterations\n";
#endif
  for(uint i=0; i< nb_iteration; i++){
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)twReal, (void*)((uintptr_t)in+sizeof(uint32_t)+ i*(sizeof(double)*FFT_SIZE)),
           (FFT_SIZE/2)*sizeof(double));
    memcpy((void*)twImg, (void*)((uintptr_t)in+sizeof(uint32_t) + sizeof(double)*(FFT_SIZE/2)+ i*(sizeof(double)*FFT_SIZE)),
           (FFT_SIZE/2)*sizeof(double));

    memcpy((void*)dReal, (void*)((uintptr_t)in+sizeof(uint32_t)+ nb_iteration *(sizeof(double)*FFT_SIZE)
                                 + i*(sizeof(double)*(2*FFT_SIZE))), (FFT_SIZE)*sizeof(double));
    memcpy((void*)dImg, (void*)((uintptr_t)in+sizeof(uint32_t) +sizeof(double)*FFT_SIZE + nb_iteration *(sizeof(double)*FFT_SIZE)
                                + i*(sizeof(double)*(2*FFT_SIZE))),(FFT_SIZE)*sizeof(double));
#else
    twReal = (double*)((uintptr_t)in+sizeof(uint32_t)
                       + i*(sizeof(double)*FFT_SIZE));
    twImg = (double*)((uintptr_t)in+sizeof(uint32_t) + sizeof(double)*(FFT_SIZE/2)
                       + i*(sizeof(double)*FFT_SIZE));

    dReal = (double*)((uintptr_t)in+sizeof(uint32_t)
                       + nb_iteration *(sizeof(double)*FFT_SIZE)
                      + i*(sizeof(double)*(2*FFT_SIZE)));
    dImg = (double*)((uintptr_t)in+sizeof(uint32_t) +sizeof(double)*FFT_SIZE
                      + nb_iteration *(sizeof(double)*FFT_SIZE)
                      + i*(sizeof(double)*(2*FFT_SIZE)));
#endif

    fft(dReal, dImg, twReal, twImg);
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)((uintptr_t)in+sizeof(uint32_t)+ nb_iteration *(sizeof(double)*FFT_SIZE)
                   + i*(sizeof(double)*(2*FFT_SIZE))),(void*)dReal,  (FFT_SIZE)*sizeof(double));
    memcpy((void*)((uintptr_t)in+sizeof(uint32_t) +sizeof(double)*FFT_SIZE + nb_iteration *(sizeof(double)*FFT_SIZE)
                   + i*(sizeof(double)*(2*FFT_SIZE))),(void*)dImg, (FFT_SIZE)*sizeof(double));
#endif
  }
#ifdef MACHSUITE_DEBUG
  std::cout << "FFT_STRIDED ended\n";
#endif
  // Output write in place in buf
  localAlloc = false;
  out = (double*)((uintptr_t)in+sizeof(uint32_t)
                + nb_iteration *(sizeof(double)*FFT_SIZE));
  return(0);
}

/**
 * @brief emu-hmpsoc wrapper around machsuite fft_strided_HW function.
 *        Allocate memory then call HW component through genericIp driver.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_00+iterOffsetIN: double[FFT_SIZE/2] twiddle factor (real)
 * @params in_01+iterOffsetIN: double[FFT_SIZE/2] twiddle factor (img)
 * @params in_10+iterOffsetOUT: double[FFT_SIZE] signal (real)
 * @params in_11+iterOffsetOUT: double[FFT_SIZE] signal (img)
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int fft_strided_hw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  double *dIn = (double*)((uintptr_t)in+sizeof(uint32_t));
  double *dInOut = (double*)((uintptr_t)in+sizeof(uint32_t)
                         + nb_iteration *(sizeof(double)*FFT_SIZE));

  // get Hardware address of input data
  uintptr_t dIn_hwAddr = monitored_VtoP(dIn);
  uintptr_t dInOut_hwAddr = monitored_VtoP(dInOut);
#ifdef MACHSUITE_DEBUG
  std::cout << "FFT_STRIDED_HW started for "<< nb_iteration <<" iterations\n"
            << "dIn@ 0x"<<std::hex<<dIn_hwAddr<<"\n"
            << "dInOut@ 0x"<<std::hex<<dInOut_hwAddr<<"\n";
#endif
  //Open device and init
  int dev_fd = open(DEV_FFT_STRIDED, O_RDWR);
  uint32_t args[3] = {nb_iteration, dIn_hwAddr, dInOut_hwAddr};
  if(0>ioctl(dev_fd, IP_INIT, args)){close(dev_fd); return -1;}
  //start computation
  if(0>ioctl(dev_fd, IP_START, NULL)){close(dev_fd); return -1;}
  //close device
  close(dev_fd);

#ifdef MACHSUITE_DEBUG
  std::cout << "FFT_STRIDED_HW ended\n";
#endif
  // Output write in place in buf
  localAlloc = false;
  out = (double*) dInOut;
}

/**
 * @brief allocate input & output memory for fft_strided.
 *        generate a benchmark coherent set of random input data.
 *
 * @param iter: (uint32_t) number of iteration.
 * @param in: ptr to data input (should be set as NULL at function call)
 * @param out: ptr to data output (should be set as NULL at function call)
 */
void fft_strided_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut){
  //allocate memory
  lIn = sizeof(uint32_t) + iter * (sizeof(double)*(3*FFT_SIZE));
  lOut = iter * (sizeof(double) *(2*FFT_SIZE));

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  void* curData = in;

  //generate benchmark coherent random data
  *((uint32_t*) curData) = iter;
  curData = ((uint32_t*) curData) + 1;

  for(uint i=0; i< iter; i++){
    for(uint s=0; s< (FFT_SIZE/2); s++){ // twiddle coeff (real+Img)
      *(((double*)curData)+s)= ((double)random()/(double)(RAND_MAX));
      *(((double*)curData)+(FFT_SIZE/2)+s)= ((double)random()/(double)(RAND_MAX));
    }
    curData = ((double*) curData) + (FFT_SIZE);
  }

  for(uint i=0; i< iter; i++){
    for(uint s=0; s< (FFT_SIZE); s++){ // signal (real +Img)
      *(((double*)curData)+s)= ((double)random()/(double)(RAND_MAX));
      *(((double*)curData)+(FFT_SIZE)+s)= ((double)random()/(double)(RAND_MAX));
    }
    curData = ((double*) curData) + (2*FFT_SIZE);
  }
}


/**
 * @brief allocate input & output memory for fft_strided.
 *        generate golden inputs/outputs tests samples
 *
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params val: uninitialized ptr to golden outputs
 */
void fft_strided_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid){
  //samples array
#include"fft_strided.samples"
  //allocate memory
  lIn = sizeof(uint32_t) + (sizeof(double)*(3*FFT_SIZE));
  lOut = (sizeof(double) *(2*FFT_SIZE));

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  valid = (void*) monitored_malloc(lOut);

  //Format inputs samples and iter => in array
  void* curData = in;
  *((uint32_t*) curData) = 1;
  curData = ((uint32_t*) curData) + 1;

  for(uint s=0; s< (FFT_SIZE)/2; s++){ // twiddle coeff (real+Img)
    *(((double*)curData)+s)= fftS_samples_twidR[s];
    *(((double*)curData)+(FFT_SIZE/2)+s)= fftS_samples_twidI[s];
  }
  curData = ((double*) curData) + (FFT_SIZE);

  for(uint s=0; s< (FFT_SIZE); s++){ // signal (real +Img)
    *(((double*)curData)+s)= fftS_samples_inR[s];
    *(((double*)curData)+(FFT_SIZE)+s)= fftS_samples_inI[s];
  }
  curData = ((double*) curData) + (2*FFT_SIZE);

  //Format outputs samples => valid array
  for(uint s=0; s< (FFT_SIZE); s++){ // signal (real +Img)
    *(((double*)valid)+s)= fftS_samples_outR[s];
    *(((double*)valid)+(FFT_SIZE)+s)= fftS_samples_outI[s];
  }
}

/**
 * @brief return inputs and output dataSize
 *
 * @params iter: number of iteration
 * @params lIn: length of the generate inputs
 * @params lOut: length of the generate outputs buffers
 */
void fft_strided_getDataSize(uint iter, size_t &lIn, size_t &lOut){
  lIn = sizeof(uint32_t) + iter * (sizeof(double)*(3*FFT_SIZE));
  lOut = iter * (sizeof(double) *(2*FFT_SIZE));
}
