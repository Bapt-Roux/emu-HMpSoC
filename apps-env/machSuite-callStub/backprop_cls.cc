/**
 * @file backprop_cls.cc
 * @brief calling Stub for machsuite BACKPROP functions.
 */
#include "backprop_cls.h"
#include <iostream>

/**
 * @brief emu-hmpsoc wrapper around machsuite backprop function.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_00+iterOffsetIN: TYPE_BP[training_sets*input_dimension]: training_data.
 * @params in_01+iterOffsetIN: TYPE_BP[training_sets*possible_outputs]: training_targets.
 * @params in_10+iterOffsetOUT: TYPE_BP[input_dimension*nodes_per_layer]: weights1.
 * @params in_11+iterOffsetOUT: TYPE_BP[nodes_per_layer*nodes_per_layer]: weights2.
 * @params in_12+iterOffsetOUT: TYPE_BP[nodes_per_layer*possible_outputs]: weights3.
 * @params in_13+iterOffsetOUT: TYPE_BP[nodes_per_layer]: biases1.
 * @params in_14+iterOffsetOUT: TYPE_BP[nodes_per_layer]: biases2.
 * @params in_15+iterOffsetOUT: TYPE_BP[possible_outputs]: biases3.
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int backprop(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;

#ifdef LOCAL_CPU_BUFFER
  TYPE_BP weights1[input_dimension*nodes_per_layer], weights2[nodes_per_layer*nodes_per_layer], weights3[nodes_per_layer*possible_outputs];
  TYPE_BP biases1[nodes_per_layer], biases2[nodes_per_layer], biases3[possible_outputs];
  TYPE_BP training_data[training_sets*input_dimension], training_targets[training_sets*possible_outputs];
#else
  TYPE_BP *weights1, *weights2, *weights3;
  TYPE_BP *biases1, *biases2, *biases3;
  TYPE_BP *training_data, *training_targets;
#endif

#ifdef MACHSUITE_DEBUG
  std::cout << "BACKPROP started for "<< nb_iteration <<" iterations\n";
#endif
  for(uint i=0; i< nb_iteration; i++){
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)training_data, (void*)((uintptr_t)in+sizeof(uint32_t)
                                         + i*(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))),
           (training_sets*input_dimension)*sizeof(TYPE_BP));
    memcpy((void*)training_targets, (void*)((uintptr_t)in+sizeof(uint32_t)+ sizeof(TYPE_BP)*(training_sets * input_dimension)
                                            + i*(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))),
           (training_sets*possible_outputs)*sizeof(TYPE_BP));
#else
    training_data = (TYPE_BP*)((uintptr_t)in+sizeof(uint32_t)
                               + i*(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs))));
    training_targets = (TYPE_BP*)((uintptr_t)in+sizeof(uint32_t)+ sizeof(TYPE_BP)*(training_sets * input_dimension)
                                  + i*(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs))));
#endif

#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)weights1, (void*)((uintptr_t)in+sizeof(uint32_t)
                                    + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                                    + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                          + possible_outputs))),(input_dimension*nodes_per_layer)*sizeof(TYPE_BP));
    memcpy((void*)weights2, (void*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_BP)*(nodes_per_layer*input_dimension)
                                    + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                                    + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                          + possible_outputs))),(nodes_per_layer*nodes_per_layer)*sizeof(TYPE_BP));
    memcpy((void*)weights3, (void*)((uintptr_t)in+sizeof(uint32_t)+ sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer))
                                    + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                                    + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                          + possible_outputs))),(nodes_per_layer*possible_outputs)*sizeof(TYPE_BP));

    memcpy((void*)biases1, (void*)((uintptr_t)in+sizeof(uint32_t)
                                   + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer+possible_outputs))
                                   + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                                   + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                         + possible_outputs))),(nodes_per_layer)*sizeof(TYPE_BP));
    memcpy((void*)biases2, (void*)((uintptr_t)in+sizeof(uint32_t)
                                   + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer+possible_outputs))
                                   + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                                   + sizeof(TYPE_BP)*(nodes_per_layer)
                                   + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                         + possible_outputs))),(nodes_per_layer)*sizeof(TYPE_BP));
    memcpy((void*)biases3, (void*)((uintptr_t)in+sizeof(uint32_t)
                                   + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer+possible_outputs))
                                   + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                                   + sizeof(TYPE_BP)*(nodes_per_layer*2)
                                   + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                         + possible_outputs))),(possible_outputs)*sizeof(TYPE_BP));
#else
    weights1 = (TYPE_BP*)((uintptr_t)in+sizeof(uint32_t)
                       + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                       + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                          + possible_outputs)));
    weights2 = (TYPE_BP*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_BP)*(nodes_per_layer*input_dimension)
                          + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                          + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                          + possible_outputs)));
    weights3 = (TYPE_BP*)((uintptr_t)in+sizeof(uint32_t)+ sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer))
                       + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                       + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                          + possible_outputs)));

    biases1 = (TYPE_BP*)((uintptr_t)in+sizeof(uint32_t)
                         + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer+possible_outputs))
                         + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                         + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                               + possible_outputs)));
    biases2 = (TYPE_BP*)((uintptr_t)in+sizeof(uint32_t)
                         + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer+possible_outputs))
                         + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                         + sizeof(TYPE_BP)*(nodes_per_layer)
                         + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                               + possible_outputs)));
    biases3 = (TYPE_BP*)((uintptr_t)in+sizeof(uint32_t)
                         + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer+possible_outputs))
                         + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                         + sizeof(TYPE_BP)*(nodes_per_layer*2)
                         + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                               + possible_outputs)));
#endif

    backprop(weights1, weights2, weights3, biases1, biases2, biases3,
             training_data, training_targets);

#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)((uintptr_t)in+sizeof(uint32_t)
                                    + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                                    + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                          + possible_outputs))), (void*)weights1, (input_dimension*nodes_per_layer)*sizeof(TYPE_BP));
    memcpy((void*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_BP)*(nodes_per_layer*input_dimension)
                                    + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                                    + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                          + possible_outputs))),(void*)weights2, (nodes_per_layer*nodes_per_layer)*sizeof(TYPE_BP));
    memcpy((void*)((uintptr_t)in+sizeof(uint32_t)+ sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer))
                                    + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                                    + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                          + possible_outputs))),(void*)weights3, (nodes_per_layer*possible_outputs)*sizeof(TYPE_BP));

    memcpy((void*)((uintptr_t)in+sizeof(uint32_t)
                                   + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer+possible_outputs))
                                   + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                                   + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                         + possible_outputs))),(void*)biases1, (nodes_per_layer)*sizeof(TYPE_BP));
    memcpy((void*)((uintptr_t)in+sizeof(uint32_t)
                                   + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer+possible_outputs))
                                   + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                                   + sizeof(TYPE_BP)*(nodes_per_layer)
                                   + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                         + possible_outputs))),(void*)biases2, (nodes_per_layer)*sizeof(TYPE_BP));
    memcpy((void*)((uintptr_t)in+sizeof(uint32_t)
                                   + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer+possible_outputs))
                                   + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                                   + sizeof(TYPE_BP)*(nodes_per_layer*2)
                                   + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                         + possible_outputs))),(void*)biases3, (possible_outputs)*sizeof(TYPE_BP));
#endif
  }
#ifdef MACHSUITE_DEBUG
  std::cout << "BACKPROP ended\n";
#endif
  // Output write in place in buf
  localAlloc = false;
  out = (TYPE_BP*)((uintptr_t)in+sizeof(uint32_t)
                   + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs))));
  return(0);
}

/**
 * @brief emu-hmpsoc wrapper around machsuite backprop_HW function.
 *        Allocate memory then call HW component through genericIp driver.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_00+iterOffsetIN: TYPE_BP[training_sets*input_dimension]: training_data.
 * @params in_01+iterOffsetIN: TYPE_BP[training_sets*possible_outputs]: training_targets.
 * @params in_10+iterOffsetOUT: TYPE_BP[input_dimension*nodes_per_layer]: weights1.
 * @params in_11+iterOffsetOUT: TYPE_BP[nodes_per_layer*nodes_per_layer]: weights2.
 * @params in_12+iterOffsetOUT: TYPE_BP[nodes_per_layer*possible_outputs]: weights3.
 * @params in_13+iterOffsetOUT: TYPE_BP[nodes_per_layer]: biases1.
 * @params in_14+iterOffsetOUT: TYPE_BP[nodes_per_layer]: biases2.
 * @params in_15+iterOffsetOUT: TYPE_BP[possible_outputs]: biases3.
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int backprop_hw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  TYPE_BP *dIn = (TYPE_BP*)((uintptr_t)in+sizeof(uint32_t));
  TYPE_BP *dInOut = (TYPE_BP*)((uintptr_t)in+sizeof(uint32_t)
                               + nb_iteration *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs))));

  // get Hardware address of input data
  uintptr_t dIn_hwAddr = monitored_VtoP(dIn);
  uintptr_t dInOut_hwAddr = monitored_VtoP(dInOut);
#ifdef MACHSUITE_DEBUG
  std::cout << "BACKPROP_HW started for "<< nb_iteration <<" iterations\n"
            << "dIn@ 0x"<<std::hex<<dIn_hwAddr<<"\n"
            << "dInOut@ 0x"<<std::hex<<dInOut_hwAddr<<"\n";
#endif
  //Open device and init
  int dev_fd = open(DEV_BACKPROP, O_RDWR);
  uint32_t args[3] = {nb_iteration, dIn_hwAddr, dInOut_hwAddr};
  if(0>ioctl(dev_fd, IP_INIT, args)){close(dev_fd); return -1;}
  //start computation
  if(0>ioctl(dev_fd, IP_START, NULL)){close(dev_fd); return -1;}
  //close device
  close(dev_fd);

#ifdef MACHSUITE_DEBUG
  std::cout << "BACKPROP_HW ended\n";
#endif
  // Output write in place in buf
  localAlloc = false;
  out = (TYPE_BP*) dInOut;
}

/**
 * @brief allocate input & output memory for backprop.
 *        generate a benchmark coherent set of random input data.
 *
 * @param iter: (uint32_t) number of iteration.
 * @param in: ptr to data input (should be set as NULL at function call)
 * @param out: ptr to data output (should be set as NULL at function call)
 */
void backprop_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut){
  //allocate memory
  lIn = sizeof(uint32_t) + iter * (sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs))
                                   + sizeof(TYPE_BP) *(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                       + possible_outputs));
  lOut = iter * (sizeof(TYPE_BP) *(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs) + possible_outputs));

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  void* curData = in;

  //generate benchmark coherent random data
  *((uint32_t*) curData) = iter;
  curData = ((uint32_t*) curData) + 1;

  for(uint i=0; i< iter; i++){ //gen training dIn data
    for(uint s=0; s< (training_sets*input_dimension); s++){ // training_data
      *(((TYPE_BP*)curData)+s)= ((TYPE_BP)random()/(TYPE_BP)(RAND_MAX));
    }
    curData = ((TYPE_BP*) curData) + (training_sets*input_dimension);
    for(uint s=0; s< (training_sets*possible_outputs); s++){ // training_targets
      *(((TYPE_BP*)curData)+s)= ((TYPE_BP)random()/(TYPE_BP)(RAND_MAX));
    }
    curData = ((TYPE_BP*) curData) + (training_sets*possible_outputs);
  }

  for(uint i=0; i< iter; i++){ //gen dInOut data
    for(uint s=0; s< (input_dimension*nodes_per_layer); s++){ //weights1
      *(((TYPE_BP*)curData)+s)= ((TYPE_BP)random()/(TYPE_BP)(RAND_MAX));
    }
    curData = ((TYPE_BP*) curData) + (input_dimension*nodes_per_layer);
    for(uint s=0; s< (nodes_per_layer*nodes_per_layer); s++){ //weights2
      *(((TYPE_BP*)curData)+s)= ((TYPE_BP)random()/(TYPE_BP)(RAND_MAX));
    }
    curData = ((TYPE_BP*) curData) + (nodes_per_layer*nodes_per_layer);
    for(uint s=0; s< (nodes_per_layer*possible_outputs); s++){ //weights3
      *(((TYPE_BP*)curData)+s)= ((TYPE_BP)random()/(TYPE_BP)(RAND_MAX));
    }
    curData = ((TYPE_BP*) curData) + (nodes_per_layer*possible_outputs);

    for(uint s=0; s< (nodes_per_layer); s++){ //biases1
      *(((TYPE_BP*)curData)+s)= ((TYPE_BP)random()/(TYPE_BP)(RAND_MAX));
    }
    curData = ((TYPE_BP*) curData) + (nodes_per_layer);
    for(uint s=0; s< (nodes_per_layer); s++){ //biases2
      *(((TYPE_BP*)curData)+s)= ((TYPE_BP)random()/(TYPE_BP)(RAND_MAX));
    }
    curData = ((TYPE_BP*) curData) + (nodes_per_layer);
    for(uint s=0; s< (possible_outputs); s++){ //biases3
      *(((TYPE_BP*)curData)+s)= ((TYPE_BP)random()/(TYPE_BP)(RAND_MAX));
    }
    curData = ((TYPE_BP*) curData) + (possible_outputs);
  }
}

/**
 * @brief allocate input & output memory for backprop.
 *        generate golden inputs/outputs tests samples
 *
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params val: uninitialized ptr to golden outputs
 */
void backprop_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid){
  //samples array
#include"backprop.samples"

  //allocate memory
  lIn = sizeof(uint32_t) + (sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs))
                            + sizeof(TYPE_BP) *(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                + possible_outputs));
  lOut = (sizeof(TYPE_BP) *(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs) + possible_outputs));

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  valid = (void*) monitored_malloc(lOut);

  //Format inputs samples and iter => in array
  void* curData = in;
  *((uint32_t*) curData) = 1;
  curData = ((uint32_t*) curData) + 1;

  for(uint s=0; s< (training_sets*input_dimension); s++){ // training_data
    *(((TYPE_BP*)curData)+s)= backprop_samples_trainingData[s];
  }
  curData = ((TYPE_BP*) curData) + (training_sets*input_dimension);
  for(uint s=0; s< (training_sets*possible_outputs); s++){ // training_targets
    *(((TYPE_BP*)curData)+s)= backprop_samples_trainingTarget[s];
  }
  curData = ((TYPE_BP*) curData) + (training_sets*possible_outputs);

  for(uint s=0; s< (input_dimension*nodes_per_layer); s++){ //weights1
    *(((TYPE_BP*)curData)+s)= backprop_samples_weight1[s];
  }
  curData = ((TYPE_BP*) curData) + (input_dimension*nodes_per_layer);
  for(uint s=0; s< (nodes_per_layer*nodes_per_layer); s++){ //weights2
    *(((TYPE_BP*)curData)+s)= backprop_samples_weight2[s];
  }
  curData = ((TYPE_BP*) curData) + (nodes_per_layer*nodes_per_layer);
  for(uint s=0; s< (nodes_per_layer*possible_outputs); s++){ //weights3
    *(((TYPE_BP*)curData)+s)= backprop_samples_weight3[s];
  }
  curData = ((TYPE_BP*) curData) + (nodes_per_layer*possible_outputs);

  for(uint s=0; s< (nodes_per_layer); s++){ //biases1
    *(((TYPE_BP*)curData)+s)= backprop_samples_biases1[s];
  }
  curData = ((TYPE_BP*) curData) + (nodes_per_layer);
  for(uint s=0; s< (nodes_per_layer); s++){ //biases2
    *(((TYPE_BP*)curData)+s)= backprop_samples_biases2[s];
  }
  curData = ((TYPE_BP*) curData) + (nodes_per_layer);
  for(uint s=0; s< (possible_outputs); s++){ //biases3
    *(((TYPE_BP*)curData)+s)= backprop_samples_biases3[s];
  }
  curData = ((TYPE_BP*) curData) + (possible_outputs);

  //Format outputs samples => valid array
  curData = valid;
  for(uint s=0; s< (input_dimension*nodes_per_layer); s++){ //weights1
    *(((TYPE_BP*)curData)+s)= backprop_samples_outW1[s];
  }
  curData = ((TYPE_BP*) curData) + (input_dimension*nodes_per_layer);
  for(uint s=0; s< (nodes_per_layer*nodes_per_layer); s++){ //weights2
    *(((TYPE_BP*)curData)+s)= backprop_samples_outW2[s];
  }
  curData = ((TYPE_BP*) curData) + (nodes_per_layer*nodes_per_layer);
  for(uint s=0; s< (nodes_per_layer*possible_outputs); s++){ //weights3
    *(((TYPE_BP*)curData)+s)= backprop_samples_outW3[s];
  }
  curData = ((TYPE_BP*) curData) + (nodes_per_layer*possible_outputs);

  for(uint s=0; s< (nodes_per_layer); s++){ //biases1
    *(((TYPE_BP*)curData)+s)= backprop_samples_outB1[s];
  }
  curData = ((TYPE_BP*) curData) + (nodes_per_layer);
  for(uint s=0; s< (nodes_per_layer); s++){ //biases2
    *(((TYPE_BP*)curData)+s)= backprop_samples_outB2[s];
  }
  curData = ((TYPE_BP*) curData) + (nodes_per_layer);
  for(uint s=0; s< (possible_outputs); s++){ //biases3
    *(((TYPE_BP*)curData)+s)= backprop_samples_outB3[s];
  }
}

/**
 * @brief return inputs and output dataSize
 *
 * @params iter: number of iteration
 * @params lIn: length of the generate inputs
 * @params lOut: length of the generate outputs buffers
 */
void backprop_getDataSize(uint iter, size_t &lIn, size_t &lOut){
  lIn = sizeof(uint32_t) + iter * (sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs))
                                   + sizeof(TYPE_BP) *(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                                       + possible_outputs));
  lOut = iter * (sizeof(TYPE_BP) *(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs) + possible_outputs));

}
