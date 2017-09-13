/**
 * @file viterbi_cls.cc
 * @brief Implementation of emu-hmpsoc viterbi calling stubs.
 * Body of functions extracted from <a href=https://github.com/breagen/MachSuite> MachSuite</a> benchmarks.
 */

#include "viterbi_cls.h"

#include <iostream>
/**
 * @brief emu-hmpsoc wrapper around machsuite viterbi function.
 * @param in_0: (uint32_t) number of iteration.
 * @param in_1+iterOffset: (tok_t[N_OBS]) observation vector.
 * @param in_2+iterOffset: (prob_t[N_STATES]) initial state probabilities.
 * @param in_3+iterOffset:(prob_t[N_STATES*N_STATES]) transition matrix.
 * @param in_4+iterOffset:(prob_t[N_STATES*N_TOKENS]) emission matrix.
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int viterbi(void* in, void* &out, bool &localAlloc){
  int status=0;
  uint32_t nb_iteration = *(uint32_t*)in;
  state_t *gb_path = (state_t*) monitored_malloc(N_OBS*nb_iteration*sizeof(state_t));

#ifdef LOCAL_CPU_BUFFER
  tok_t obs[N_OBS];
  prob_t init[N_STATES], transition[N_STATES*N_STATES], emission[N_STATES*N_TOKENS];
  state_t loc_path[N_OBS];
#else
  tok_t *obs;
  prob_t *init, *transition, *emission;
  state_t *loc_path;
#endif

#ifdef MACHSUITE_DEBUG
  std::cout << "VITERBI started for "<< nb_iteration <<" iterations\n";
#endif
  for(int i=0; i< nb_iteration; i++){
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)obs, (void*) ((uintptr_t) in +sizeof(uint32_t)
                                + i*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)))),
           N_OBS*sizeof(tok_t));
    memcpy((void*) init, (void*) ((uintptr_t) in +sizeof(uint32_t)+ sizeof(tok_t)*N_OBS
                                  + i*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)))),
           N_STATES*sizeof(prob_t));
    memcpy((void*)transition, (void*) ((uintptr_t) in +sizeof(uint32_t)+ sizeof(tok_t)*N_OBS + sizeof(prob_t)*N_STATES
                                       + i*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)))),
           (N_STATES*N_STATES)*sizeof(prob_t));
    memcpy((void*)emission, (void*) ((uintptr_t) in +sizeof(uint32_t)+ sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES ))
                                     + i*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)))),
           (N_STATES*N_TOKENS)*sizeof(prob_t));

#else
    obs = (tok_t*) ((uintptr_t) in +sizeof(uint32_t)
                    + i*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS))));
    init = (prob_t*) ((uintptr_t) in +sizeof(uint32_t)
                      + i*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)))
                      + sizeof(tok_t)*N_OBS);
    transition = (prob_t*) ((uintptr_t) in +sizeof(uint32_t)
                            + i*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)))
                            + sizeof(tok_t)*N_OBS + sizeof(prob_t)*N_STATES);
    emission = (prob_t*) ((uintptr_t) in +sizeof(uint32_t)
                          + i*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)))
                          + sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES )));
    loc_path = (state_t*) ((uintptr_t) gb_path + i*N_OBS*sizeof(state_t));
#endif
    status += viterbi(obs, init, transition, emission, loc_path);

#ifdef LOCAL_CPU_BUFFER
    memcpy((void*) ((uintptr_t) gb_path + i*N_OBS*sizeof(state_t)), (void*) loc_path, N_OBS*sizeof(state_t));
#endif
  }
#ifdef MACHSUITE_DEBUG
  std::cout << "VITERBI ended\n";
#endif
  // Output write in place in buf
  localAlloc = true;
  out = (void*) gb_path;
  return status;
}


/**
 * @brief emu-hmpsoc wrapper around machsuite viterbi_HW function.
 *        Allocate memory then call HW component through genericIp driver.
 *
 * @param in_0: (uint32_t) number of iteration.
 * @param in_1+iterOffset: (tok_t[N_OBS]) observation vector.
 * @param in_2+iterOffset: (prob_t[N_STATES]) initial state probabilities.
 * @param in_3+iterOffset:(prob_t[N_STATES*N_STATES]) transition matrix.
 * @param in_4+iterOffset:(prob_t[N_STATES*N_TOKENS]) emission matrix.
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int viterbi_hw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  uint8_t *dataIn = (uint8_t*) ((uintptr_t)in+sizeof(uint32_t));
  state_t *path = (state_t*) monitored_malloc(N_OBS*nb_iteration*sizeof(state_t));

  // get Hardware address of input/output data
  uintptr_t dataIn_hwAddr = monitored_VtoP(dataIn);
  uintptr_t path_hwAddr = monitored_VtoP(path);


#ifdef MACHSUITE_DEBUG
  std::cout << "VITERBI_HW started for "<< nb_iteration <<" iterations\n"
            << "dataIn @0x"<<std::hex<<dataIn_hwAddr<<"\n"
            << "path @0x"<<std::hex<<path_hwAddr<<"\n";
#endif
  //Open device and init
  int dev_fd = open(DEV_VITERBI, O_RDWR);
  uint32_t args[3] = {nb_iteration, dataIn_hwAddr, path_hwAddr};
  if(0>ioctl(dev_fd, IP_INIT, args)){close(dev_fd); return -1;}
  //start computation
  if(0>ioctl(dev_fd, IP_START, NULL)){close(dev_fd); return -1;}
  //close device
  close(dev_fd);

#ifdef MACHSUITE_DEBUG
  std::cout << "VITERBI_HW end.\n";
#endif
  // Output write in place in buf
  localAlloc = true;
  out = (state_t*) path;
  return(0);
}

/**
 * @brief allocate input & output memory for viterbi.
 *        generate a benchmark coherent set of random input data.
 *
 * @param iter: (uint32_t) number of iteration.
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 */
void viterbi_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut){
  //allocate memory
 lIn= sizeof(uint32_t) + iter*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)));
 lOut = (iter*sizeof(state_t)*N_OBS);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  void* curData = in;

  //generate benchmark coherent random data
  uint seed;
  TYPE fseed;
  *((uint32_t*) curData) = iter;
  curData = ((uint32_t*) curData) + 1;

  for(uint i=0; i< iter; i++){
    //gen obs values
    seed = random()%64;
    bool isGrow = random()%2;
    for(uint o=0; o<N_OBS; o++){
      seed = isGrow?(seed + random()%64):random()%64; //inc or restart ?
      *(((tok_t*)curData)+o) = seed;
    }
    curData = ((tok_t*) curData) + N_OBS;

    //gen init values
    fseed = ((TYPE)random()/(TYPE)(RAND_MAX))*15;
    for(uint i=0; i<N_STATES; i++){
      fseed += ((TYPE)random()/(TYPE)(RAND_MAX))*4;
      *(((prob_t*)curData)+i) = fseed;
    }
    curData = ((prob_t*) curData) + N_STATES;

    //gen transition values
    fseed = ((TYPE)random()/(TYPE)(RAND_MAX))*25;
    for(uint i=0; i<(N_STATES*N_STATES); i++){
      fseed += ((TYPE)random()/(TYPE)(RAND_MAX))*10;
      *(((prob_t*)curData)+i) = fseed;
    }
    curData = ((prob_t*) curData) + (N_STATES*N_STATES);

    //gen emission values
    fseed = ((TYPE)random()/(TYPE)(RAND_MAX))*25;
    for(uint i=0; i<(N_STATES*N_TOKENS); i++){
      fseed += ((TYPE)random()/(TYPE)(RAND_MAX))*10;
      *(((prob_t*)curData)+i) = fseed;
    }
    curData = ((prob_t*) curData) + (N_STATES*N_TOKENS);
  }
}



/**
 * @brief allocate input & output memory for viterbi.
 *        generate golden inputs/outputs tests samples
 *
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params val: uninitialized ptr to golden outputs
 */
void viterbi_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid){
  //samples array
  #include "viterbi.samples"
  //allocate memory
  lIn= sizeof(uint32_t) + (sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)));
  lOut = (sizeof(state_t)*N_OBS);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  valid = (void*) monitored_malloc(lOut);

  //Format inputs samples and iter => in array
  void* curData = in;
  *((uint32_t*) curData) = 1;
  curData = ((uint32_t*) curData) + 1;

  for(uint i=0; i<N_OBS; i++){//gen obs value
    *(((tok_t*)curData)+i) = viterbi_samples_obs[i];
  }
  curData = ((tok_t*) curData) + N_OBS;

  //gen init values
  for(uint i=0; i<N_STATES; i++){
    *(((prob_t*)curData)+i) = viterbi_samples_init[i];
  }
  curData = ((prob_t*) curData) + N_STATES;

  //gen transition values
  for(uint i=0; i<(N_STATES*N_STATES); i++){
    *(((prob_t*)curData)+i) = viterbi_samples_trans[i];
  }
  curData = ((prob_t*) curData) + (N_STATES*N_STATES);

  //gen emission values
  for(uint i=0; i<(N_STATES*N_TOKENS); i++){
    *(((prob_t*)curData)+i) = viterbi_samples_emission[i];
  }
  curData = ((prob_t*) curData) + (N_STATES*N_TOKENS);

  //Format outputs samples => valid array
  for(uint i=0; i<(lOut); i++){
    *((uint8_t*)valid+i) = viterbi_samples_out[i];
  }
}

/**
 * @brief return inputs and output dataSize
 *
 * @params iter: number of iteration
 * @params lIn: length of the generate inputs
 * @params lOut: length of the generate outputs buffers
 */
void viterbi_getDataSize(uint iter, size_t &lIn, size_t &lOut){
  lIn= sizeof(uint32_t) + iter*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)));
  lOut = (iter*sizeof(state_t)*N_OBS);
}
