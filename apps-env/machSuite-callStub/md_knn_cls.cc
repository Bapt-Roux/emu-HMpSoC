/**
 * @file md_knn_cls.cc
 * @brief calling Stub for machsuite MD_KNN functions.
 */
#include "md_knn_cls.h"
#include <iostream>

/**
 * @brief emu-hmpsoc wrapper around machsuite md_knn function.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1+iterOffset: TYPE_MDK[nAtoms] X positions
 * @params in_2+iterOffset: TYPE_MDK[nAtoms] Y positions
 * @params in_3+iterOffset: TYPE_MDK[nAtoms] Z positions
 * @params in_4+iterOffset: int32_t[nAtoms * maxNeighbors] neighbors list
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int md_knn(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  TYPE_MDK *dOut = (TYPE_MDK*) monitored_malloc(nb_iteration*sizeof(TYPE_MDK)*(3* nAtoms));
#ifdef LOCAL_CPU_BUFFER
  TYPE_MDK xp[nAtoms], yp[nAtoms], zp[nAtoms];
  int32_t neighbors[nAtoms * maxNeighbors];
  TYPE_MDK xf[nAtoms], yf[nAtoms], zf[nAtoms];
#else
  TYPE_MDK *xp, *yp, *zp;
  int32_t *neighbors;
  TYPE_MDK *xf, *yf, *zf;
#endif

#ifdef MACHSUITE_DEBUG
  std::cout << "MD_KNN started for "<< nb_iteration <<" iterations\n";
#endif
  for(uint i=0; i< nb_iteration; i++){
#ifdef LOCAL_CPU_BUFFER
    //inputs
    memcpy((void*)xp, (void*)((uintptr_t)in+sizeof(uint32_t) + i*(sizeof(TYPE_MDK)*(3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors))),
           nAtoms*sizeof(TYPE_MDK));
    memcpy((void*)yp, (void*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_MDK)*(nAtoms)+ i*(sizeof(TYPE_MDK)*(3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors))),
           nAtoms*sizeof(TYPE_MDK));
    memcpy((void*)zp, (void*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_MDK)*(2*nAtoms) + i*(sizeof(TYPE_MDK)*(3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors))),
           nAtoms*sizeof(TYPE_MDK));
    memcpy((void*)neighbors, (void*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_MDK)*(3*nAtoms)+ i*(sizeof(TYPE_MDK)*(3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors))),
           (nAtoms*maxNeighbors)*sizeof(int32_t));
#else
    //inputs
    xp = (TYPE_MDK*)((uintptr_t)in+sizeof(uint32_t)
                 + i*(sizeof(TYPE_MDK)*(3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors)));
    yp = (TYPE_MDK*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_MDK)*(nAtoms)
                 + i*(sizeof(TYPE_MDK)*(3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors)));
    zp = (TYPE_MDK*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_MDK)*(2*nAtoms)
                 + i*(sizeof(TYPE_MDK)*(3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors)));
    neighbors = (int32_t*)((uintptr_t)in+sizeof(uint32_t) + sizeof(TYPE_MDK)*(3*nAtoms)
                 + i*(sizeof(TYPE_MDK)*(3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors)));
    //outputs
    xf = (TYPE_MDK*)((uintptr_t)dOut
                 + i*(sizeof(TYPE_MDK)*(3*nAtoms)));
    yf = (TYPE_MDK*)((uintptr_t)dOut + sizeof(TYPE_MDK)*(nAtoms)
                 + i*(sizeof(TYPE_MDK)*(3*nAtoms)));
    zf = (TYPE_MDK*)((uintptr_t)dOut + sizeof(TYPE_MDK)*(2*nAtoms)
                 + i*(sizeof(TYPE_MDK)*(3*nAtoms)));
#endif
    md_kernel(xf, yf ,zf,
              xp, yp, zp, neighbors);
#ifdef LOCAL_CPU_BUFFER
    memcpy((void*)((uintptr_t)dOut+ i*(sizeof(TYPE_MDK)*(3*nAtoms))),(void*)xf,nAtoms*sizeof(TYPE_MDK));
    memcpy((void*)((uintptr_t)dOut+ sizeof(TYPE_MDK)*(nAtoms)+ i*(sizeof(TYPE_MDK)*(3*nAtoms))),(void*)yf,nAtoms*sizeof(TYPE_MDK));
    memcpy((void*)((uintptr_t)dOut+ sizeof(TYPE_MDK)*(2*nAtoms) + i*(sizeof(TYPE_MDK)*(3*nAtoms))),(void*)zf,nAtoms*sizeof(TYPE_MDK));
#endif
  }
#ifdef MACHSUITE_DEBUG
  std::cout << "MD_KNN ended\n";
#endif
  // Output write in place in buf
  localAlloc = true;
  out = (TYPE_MDK*)(dOut);
  return(0);
}

/**
 * @brief emu-hmpsoc wrapper around machsuite md_knn_HW function.
 *        Allocate memory then call HW component through genericIp driver.
 * @param in_0: (uint32_t) number of iteration.
 * @params in_1+iterOffset: TYPE_MDK[nAtoms] X positions
 * @params in_2+iterOffset: TYPE_MDK[nAtoms] Y positions
 * @params in_3+iterOffset: TYPE_MDK[nAtoms] Z positions
 * @params in_4+iterOffset: int32_t[nAtoms * maxNeighbors] neighbors list
 *
 * @param out: pointer to output data
 * @param localAlloc: false if output data was write in place
 *
 * @return negative number on failure.
 */
int md_knn_hw(void* in, void* &out, bool &localAlloc){
  uint32_t nb_iteration = *(uint32_t*)in;
  TYPE_MDK *dIn = (TYPE_MDK*)((uintptr_t)in+sizeof(uint32_t));
  TYPE_MDK *dOut = (TYPE_MDK*) monitored_malloc(nb_iteration*sizeof(TYPE_MDK)*(3*nAtoms));


  // get Hardware address of input data
  uintptr_t dIn_hwAddr = monitored_VtoP(dIn);
  uintptr_t dOut_hwAddr = monitored_VtoP(dOut);
#ifdef MACHSUITE_DEBUG
  std::cout << "MD_KNN_HW started for "<< nb_iteration <<" iterations\n"
            << "dIn@ 0x"<<std::hex<<dIn_hwAddr<<"\n"
            << "dOut@ 0x"<<std::hex<<dOut_hwAddr<<"\n";
#endif
  //Open device and init
  int dev_fd = open(DEV_MD_KNN, O_RDWR);
  uint32_t args[3] = {nb_iteration, dIn_hwAddr, dOut_hwAddr};
  if(0>ioctl(dev_fd, IP_INIT, args)){close(dev_fd); return -1;}
  //start computation
  if(0>ioctl(dev_fd, IP_START, NULL)){close(dev_fd); return -1;}
  //close device
  close(dev_fd);

#ifdef MACHSUITE_DEBUG
  std::cout << "MD_KNN_HW ended\n";
#endif
  // Output write in place in buf
  localAlloc = true;
  out = (TYPE_MDK*) dOut;
}

/**
 * @brief allocate input & output memory for md_knn.
 *        generate a benchmark coherent set of random input data.
 *
 * @param iter: (uint32_t) number of iteration.
 * @param in: ptr to data input (should be set as NULL at function call)
 * @param out: ptr to data output (should be set as NULL at function call)
 */
void md_knn_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut){
  //allocate memory
  lIn = sizeof(uint32_t) + iter * (sizeof(TYPE_MDK)*(3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors));
  lOut = iter * (sizeof(TYPE_MDK) *(3*nAtoms));

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  void* curData = in;

  //generate benchmark coherent random data
  *((uint32_t*) curData) = iter;
  curData = ((uint32_t*) curData) + 1;

  for(uint i=0; i< iter; i++){
    for(uint s=0; s< 3*nAtoms; s++){ //positive double between 0 and 30
      *(((TYPE_MDK*)curData)+s)= (TYPE_MDK)((uint64_t)(((TYPE_MDK)random()/(TYPE_MDK)(RAND_MAX))*30) & 0x7fffffffffffffff);
    }
    curData = ((TYPE_MDK*) curData) + (3*nAtoms);

    for(uint s=0; s<(nAtoms*maxNeighbors); s++){//positive int between 0 and 300
      *(((int32_t*)curData)+s)= (int32_t)(((int32_t)random()%300) & 0x7fffffff);
    }
    curData = ((TYPE_MDK*) curData) + (nAtoms*maxNeighbors);
  }
}

/**
 * @brief allocate input & output memory for md_knn.
 *        generate golden inputs/outputs tests samples
 *
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params val: uninitialized ptr to golden outputs
 */
void md_knn_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid){
  //samples array
  #include"md_knn.samples"

  //allocate memory
  lIn = sizeof(uint32_t) + (sizeof(TYPE_MDK)*(3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors));
  lOut = (sizeof(TYPE_MDK) *(3*nAtoms));

  in = (void*) monitored_malloc(lIn);
  out = (void*) monitored_malloc(lOut);
  valid = (void*) monitored_malloc(lOut);

  //Format inputs samples and iter => in array
  void* curData = in;

  *((uint32_t*) curData) = 1;
  curData = ((uint32_t*) curData) + 1;

  for(uint s=0; s< nAtoms; s++){
    *(((TYPE_MDK*)curData)+s)= mdK_samples_px[s];
  }
  curData = ((TYPE_MDK*) curData) + (nAtoms);
  for(uint s=0; s< nAtoms; s++){
    *(((TYPE_MDK*)curData)+s)= mdK_samples_py[s];
  }
  curData = ((TYPE_MDK*) curData) + (nAtoms);
    for(uint s=0; s< nAtoms; s++){
      *(((TYPE_MDK*)curData)+s)= mdK_samples_pz[s];
    }
    curData = ((TYPE_MDK*) curData) + (nAtoms);

    for(uint s=0; s<(nAtoms*maxNeighbors); s++){
      *(((int32_t*)curData)+s)= mdK_samples_neighbor[s];
    }
  //Format outputs samples => valid array
    curData = valid;
    for(uint s=0; s< nAtoms; s++){
      *(((TYPE_MDK*)curData)+s)= mdK_samples_outFx[s];
    }
    curData = ((TYPE_MDK*) curData) + (nAtoms);
    for(uint s=0; s< nAtoms; s++){
      *(((TYPE_MDK*)curData)+s)= mdK_samples_outFy[s];
    }
    curData = ((TYPE_MDK*) curData) + (nAtoms);
    for(uint s=0; s< nAtoms; s++){
      *(((TYPE_MDK*)curData)+s)= mdK_samples_outFz[s];
    }
}

/**
 * @brief return inputs and output dataSize
 *
 * @params iter: number of iteration
 * @params lIn: length of the generate inputs
 * @params lOut: length of the generate outputs buffers
 */
void md_knn_getDataSize(uint iter, size_t &lIn, size_t &lOut){
  lIn = sizeof(uint32_t) + iter * (sizeof(TYPE_MDK)*(3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors));
  lOut = iter * (sizeof(TYPE_MDK) *(3*nAtoms));
}
