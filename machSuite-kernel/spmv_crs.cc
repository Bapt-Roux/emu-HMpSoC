/**
 * @file spmv_crs.cc
 * @brief Implementation of emu-hmpsoc SPMV_CRS functions.
 * Body of functions extracted from <a href=https://github.com/breagen/MachSuite> MachSuite</a> benchmarks.
 */
#include "spmv_crs.h"

/* ----------------- Body function imported from machsuite. ----------------- */
/*
  Based on algorithm described here:
  http://www.cs.berkeley.edu/~mhoemmen/matrix-seminar/slides/UCB_sparse_tutorial_1.pdf
*/

void spmv(TYPE_SPMVC val[NNZ_SPMVC], int32_t cols[NNZ_SPMVC], int32_t rowDelimiters[N_SPMVC+1], TYPE_SPMVC vec[N_SPMVC], TYPE_SPMVC out[N_SPMVC]){
  int i, j;
  TYPE_SPMVC sum, Si;

 spmv_1 : for(i = 0; i < N_SPMVC; i++){
    sum = 0; Si = 0;
    int tmp_begin = rowDelimiters[i];
    int tmp_end = rowDelimiters[i+1];
  spmv_2 : for (j = tmp_begin; j < tmp_end; j++){
      Si = val[j] * vec[cols[j]];
      sum = sum + Si;
    }
    out[i] = sum;
  }
}


/* -------------------------------------------------------------------------- */
