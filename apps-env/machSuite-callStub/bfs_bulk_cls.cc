/**
 * @file bfs_bulk_cls.cc
 * @brief Implementation of emu-hmpsoc bfs_bulk calling stubs.
 * Body of functions extracted from <a href=https://github.com/breagen/MachSuite> MachSuite</a> benchmarks.
 */

#include "bfs_bulk_cls.h"

/**
 * @brief emu-hmpsoc wrapper around machsuite viterbi function.
 * @param in_0: (uint32_t) number of iteration.
 * @param in_1+iterOffset: (node_index_t) starting node
 * @param in_2+iterOffset: (node_t[N_NODES]) node structures (start and end indices of edge lists)
 * @param in_3+iterOffset: (edge_t[N_EDGES]) edges structures (just destination node id)
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int bfs_bulk(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  edge_index_t *gb_lvl_counts = (edge_index_t*) monitored_malloc((nb_iteration*N_LEVELS)*sizeof(edge_index_t));
  node_index_t *starting_node;

#ifdef LOCAL_CPU_BUFFER
  node_t nodes[N_NODES];
  edge_t edges[N_EDGES];
  edge_index_t loc_lvl_counts[N_LEVELS];
  level_t lvl[N_NODES];
#else
  node_t *nodes;
  edge_t *edges;
  edge_index_t *loc_lvl_counts;
  level_t *lvl = (level_t*) monitored_malloc(N_NODES*sizeof(level_t));
#endif
  for(int i=0; i< nb_iteration; i++){

    for(int n=0; n<N_NODES; n++) {lvl[n]=MAX_LEVEL;} // Max-ify levels
    for(int n=0; n<N_LEVELS; n++) {loc_lvl_counts[n]=0;} // clear localBuffer
    starting_node = (node_index_t*) ((uintptr_t) in +sizeof(uint32_t)
                                     + i*(sizeof(node_index_t) + sizeof(node_t)*N_NODES + sizeof(edge_t)*N_EDGES));

#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)nodes, (void*) ((uintptr_t) in +sizeof(uint32_t)+ sizeof(node_index_t)
                                  + i*(sizeof(node_index_t) + sizeof(node_t)*N_NODES + sizeof(edge_t)*N_EDGES)),
           N_NODES*sizeof(node_t));
    memcpy((void*)edges, (void*) ((uintptr_t) in +sizeof(uint32_t)+ sizeof(node_index_t) + sizeof(node_t)*N_NODES
                                  + i*(sizeof(node_index_t) + sizeof(node_t)*N_NODES + sizeof(edge_t)*N_EDGES)),
           N_EDGES*sizeof(edge_t));
#else
    nodes = (node_t*) ((uintptr_t) in +sizeof(uint32_t)
                       + i*(sizeof(node_index_t) + sizeof(node_t)*N_NODES + sizeof(edge_t)*N_EDGES)
                       + sizeof(node_index_t));
    edges = (edge_t*) ((uintptr_t) in +sizeof(uint32_t)
                       + i*(sizeof(node_index_t) + sizeof(node_t)*N_NODES + sizeof(edge_t)*N_EDGES)
                       + (sizeof(node_index_t) + sizeof(node_t)*N_NODES));

    loc_lvl_counts = (edge_index_t*) ((uintptr_t)gb_lvl_counts + i* N_LEVELS*sizeof(edge_index_t));
#endif

    //keep starting_node within space
    *starting_node %= (N_NODES-1);
    bfs(nodes, edges, *starting_node, lvl, loc_lvl_counts);

#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)((uintptr_t)gb_lvl_counts + i* N_LEVELS*sizeof(edge_index_t)), (void*)loc_lvl_counts, N_LEVELS*sizeof(edge_index_t));
#endif
  }

  // Output write in place in buf
  localAlloc = true;
  out = (void*) gb_lvl_counts;

#ifndef LOCAL_CPU_BUFFER
  monitored_free(lvl);
#endif
  return(0);
}

/**
 * @brief emu-hmpsoc wrapper around machsuite bfs_bulk_HW function.
 *        Allocate memory then call HW component through genericIp driver.
 *
 * @param in_0: (uint32_t) number of iteration.
 * @param in_1+iterOffset: (node_index_t) starting node
 * @param in_2+iterOffset: (node_t[N_NODES]) node structures (start and end indices of edge lists)
 * @param in_3+iterOffset: (edge_t[N_EDGES]) edges structures (just destination node id)
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int bfs_bulk_hw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  uint8_t *dataIn = (uint8_t*) ((uintptr_t)in+sizeof(uint32_t));
  edge_index_t *lvl_counts = (edge_index_t*) monitored_malloc((nb_iteration*N_LEVELS)*sizeof(edge_index_t));

  // get Hardware address of input/output data
  uintptr_t dataIn_hwAddr = monitored_VtoP(dataIn);
  uintptr_t lvlCnt_hwAddr = monitored_VtoP(lvl_counts);

  //Open device and init
  int dev_fd = open(DEV_BBULK, O_RDWR);
  uint32_t args[3] = {nb_iteration, dataIn_hwAddr, lvlCnt_hwAddr};
  if(0>ioctl(dev_fd, IP_INIT, args)){close(dev_fd); return -1;}
  //start computation
  if(0>ioctl(dev_fd, IP_START, NULL)){close(dev_fd); return -1;}
  //close device
  close(dev_fd);

  // Output write in place in buf
  localAlloc = true;
  out = (edge_index_t*) lvl_counts;
  return(0);
}

/**
 * @brief allocate input & output memory for bfs_bulk.
 *        generate a benchmark coherent set of random input data.
 *
 * @param iter: (uint32_t) number of iteration.
 * @param in: ptr to data input (should be set as NULL at function call)
 * @param out: ptr to data output (should be set as NULL at function call)
 */
void bfs_bulk_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut){
  //allocate memory
  lIn = sizeof(uint32_t)+ iter*(sizeof(node_index_t) + sizeof(node_t)*N_NODES + sizeof(edge_t)*N_EDGES);
  lOut = (iter*sizeof(level_t)*N_NODES);

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  void* curData = in;

  //generate benchmark coherent random data
  uint seed;
  *((uint32_t*) curData) = iter;
  curData = ((uint32_t*) curData) + 1;

  for(uint i=0; i< iter; i++){
    // gen starting_node
    *((node_index_t*) curData) = random()%N_NODES;
    curData = ((node_index_t*) curData) + 1;

    //gen nodes values
    seed = random()%256;
    for(uint n=0; n<N_NODES; n++){
      (((node_t*)curData)+n)->edge_begin = seed;
      seed += random()%16;
      (((node_t*)curData)+n)->edge_end = seed;
    }
    curData = ((node_t*) curData) + N_NODES;

    //gen nodes values
    seed = random()%256;
    bool isGrow = random()%2;
    for(uint e=0; e<N_EDGES; e++){
      seed = isGrow?(seed + random()%128):random()%128; //inc or restart ?
      (((edge_t*)curData)+e)->dst = seed;
    }
    curData = ((edge_t*) curData) + N_EDGES;
  }
}

  /**
   * @brief allocate input & output memory for bfs_bulk.
   *        generate golden inputs/outputs tests samples
   *
   * @params in: uninitialized ptr to inputs data
   * @params lIn: length of the generate inputs
   * @params out: uninitialized ptr to outputs buffer
   * @params lOut: length of the generate outputs buffers
   * @params val: uninitialized ptr to golden outputs
   */
  void bfs_bulk_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid){
    //samples array
    #include"bfs_bulk.samples"

    //allocate memory
    lIn = sizeof(uint32_t)+ (sizeof(node_index_t) + sizeof(node_t)*N_NODES + sizeof(edge_t)*N_EDGES);
    lOut = (sizeof(edge_index_t)*N_LEVELS);

    in = (void*) monitored_malloc(lIn);
    out = (void*) monitored_malloc(lOut);
    valid = (void*) monitored_malloc(lOut);

    //Format inputs samples and iter => in array
    void* curData = in;
    *((uint32_t*) curData) = 1;
    curData = ((uint32_t*) curData) + 1;

    // gen starting_node
    *((node_index_t*) curData) = bbfs_samples_startN;
    curData = ((node_index_t*) curData) + 1;

    //gen nodes values
    for(uint n=0; n<N_NODES; n++){
      (((node_t*)curData)+n)->edge_begin = (edge_index_t) bbfs_samples_nodes[2*n];
      (((node_t*)curData)+n)->edge_end = (edge_index_t) bbfs_samples_nodes[(2*n)+1];
    }
    curData = ((node_t*) curData) + N_NODES;

    //gen nodes values
    for(uint e=0; e<N_EDGES; e++){
      (((edge_t*)curData)+e)->dst = (node_index_t) bbfs_samples_edges[e];
    }
//Format outputs samples => valid array
    for(uint n=0; n<N_LEVELS; n++){
      *(((uint64_t*)valid)+n) = bbfs_samples_outputs[n];
    }
  }

/**
 * @brief return inputs and output dataSize
 *
 * @params iter: number of iteration
 * @params lIn: length of the generate inputs
 * @params lOut: length of the generate outputs buffers
 */
void bfs_bulk_getDataSize(uint iter, size_t &lIn, size_t &lOut){
  lIn = sizeof(uint32_t)+ iter*(sizeof(node_index_t) + sizeof(node_t)*N_NODES + sizeof(edge_t)*N_EDGES);
  lOut = (iter*sizeof(level_t)*N_NODES);
}
