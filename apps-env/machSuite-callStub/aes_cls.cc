/**
 * @file aes_cls.cc
 * @brief calling Stub for machsuite AES functions.
 */
#include "aes_cls.h"
#include <iostream>

/**
 * @brief emu-hmpsoc wrapper around machsuite aes function.
 * @param in[0]: (uint32_t) number of iteration.
 * @param in[1..33]: (uint8_t[32]) encryption key.
 * @param in[33..]: (uint8_t[16*in[0]]) data input for each iterations.
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int aes(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  aes256_context ctx;

#ifdef LOCAL_CPU_BUFFER
  uint8_t key[32], buf[16];
  memcpy((void*)key, (void*) ((uintptr_t)in+sizeof(uint32_t)), sizeof(uint8_t)*32);
#else
  uint8_t *key = (uint8_t*) ((uintptr_t)in+sizeof(uint32_t));
  uint8_t *buf;
#endif

#ifdef MACHSUITE_DEBUG
  std::cout << "AES started for "<< nb_iteration <<" iterations\n";
#endif
  for(uint i=0; i< nb_iteration; i++){
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)buf, (void*)((uintptr_t)in+sizeof(uint32_t)+ 32*sizeof(uint8_t)
                               + i*16*sizeof(uint8_t)), 16*sizeof(uint8_t));
#else
    buf = (uint8_t*)((uintptr_t)in+sizeof(uint32_t)+ 32*sizeof(uint8_t)
                     + i*16*sizeof(uint8_t));
#endif
    aes256_encrypt_ecb(&ctx, key, buf);

#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)((uintptr_t)in+sizeof(uint32_t)+ 32*sizeof(uint8_t) + i*16*sizeof(uint8_t)),
           (void*)buf, 16*sizeof(uint8_t));
#endif
  }
#ifdef MACHSUITE_DEBUG
  std::cout << "AES ended\n";
#endif
  // Output write in place in buf
  localAlloc = false;
  out = (int8_t*)(in+sizeof(uint32_t)+ 32*sizeof(int8_t));
  return(0);
}

/**
 * @brief emu-hmpsoc wrapper around machsuite aes_HW function.
 *        Allocate memory then call HW component through genericIp driver.
 * @param in[0]: (uint32_t) number of iteration.
 * @param in[1..33]: (uint8_t[32]) encryption key.
 * @param in[33..]: (uint8_t[16*in[0]]) data input for each iterations.
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int aes_hw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  uint8_t *key = (uint8_t*) ((uintptr_t)in+sizeof(uint32_t));
  uint8_t *buf = (uint8_t*)((uintptr_t)in+sizeof(uint32_t)+ 32*sizeof(uint8_t));

  // get Hardware address of input data
  uintptr_t key_hwAddr = monitored_VtoP(key);
  uintptr_t buf_hwAddr = monitored_VtoP(buf);
#ifdef MACHSUITE_DEBUG
  std::cout << "AES_HW started for "<< nb_iteration <<" iterations\n"
            << "key@ 0x"<<std::hex<<key_hwAddr<<"\n"
            << "buf@ 0x"<<std::hex<<buf_hwAddr<<"\n";
#endif
  //Open device and init
  int dev_fd = open(DEV_AES, O_RDWR);
  uint32_t args[3] = {nb_iteration, key_hwAddr, buf_hwAddr};
  if(0>ioctl(dev_fd, IP_INIT, args)){close(dev_fd); return -1;}
  //start computation
  if(0>ioctl(dev_fd, IP_START, NULL)){close(dev_fd); return -1;}
  //close device
  close(dev_fd);

#ifdef MACHSUITE_DEBUG
  std::cout << "AES_HW ended\n";
#endif
  // Output write in place in buf
  localAlloc = false;
  out = (uint8_t*) buf;
}

/**
 * @brief allocate input & output memory for aes.
 *        generate a benchmark coherent set of random input data.
 *
 * @param iter: (uint32_t) number of iteration.
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 */
void aes_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut){
  srand(time(NULL));
  //allocate memory
  lIn = (32+ iter*16)*sizeof(uint8_t) +sizeof(uint32_t);
  lOut = (iter*16)*sizeof(uint8_t);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  void* curData = in;

  //generate benchmark coherent random data
  *((uint32_t*) curData) = iter;
  curData = ((uint32_t*) curData) + 1;
  for(uint k=0; k< 32; k++){ // generate key
    *(((uint8_t*)curData)+k)= random();
  }
  curData = ((uint8_t*) curData) + 32;

  for(uint i=0; i< iter; i++){
    for(uint b=0; b< 32; b++){ // gene data to encode
      *(((uint8_t*)curData)+b)= random();
    }
    curData = ((uint8_t*) curData) + 16;
  }
}

/**
 * @brief allocate input & output memory for aes.
 *        generate golden inputs/outputs tests samples
 *
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params val: uninitialized ptr to golden outputs
 */
void aes_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid){
  //samples array
  #include"aes.samples"

  //allocate memory
  lIn = (32+ 16)*sizeof(uint8_t) +sizeof(uint32_t);
  lOut = 16*sizeof(uint8_t);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  valid = (void*) monitored_malloc(lOut);

  //Format inputs samples and iter => in array
  void* curData = in;
  *((uint32_t*) curData) = 1;
  curData = ((uint32_t*) curData) + 1;
  for(uint i=0; i< 32; i++){ // set key
    *(((uint8_t*)curData)+i)= aes_sample_key[i];
  }
  curData = ((uint8_t*) curData) + 32;

  for(uint i=0; i< 16; i++){ // set buf
    *(((uint8_t*)curData)+i)= aes_sample_buf[i];
  }

  //Format outputs samples => valid array
    for(uint i=0; i< 16; i++){ // gene data to encode
      *(((uint8_t*)valid)+i)= aes_sample_out[i];
    }
}

/**
 * @brief return inputs and output dataSize
 *
 * @params iter: number of iteration
 * @params lIn: length of the generate inputs
 * @params lOut: length of the generate outputs buffers
 */
void aes_getDataSize(uint iter, size_t &lIn, size_t &lOut){
  lIn = (32+ iter*16)*sizeof(uint8_t) +sizeof(uint32_t);
  lOut = (iter*16)*sizeof(uint8_t);
}
