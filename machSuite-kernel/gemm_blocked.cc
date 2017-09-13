/**
 * @file gemm_blocked.cc
 * @brief Implementation of emu-hmpsoc GEMM_BLOCKED functions.
 * Body of functions extracted from <a href=https://github.com/breagen/MachSuite> MachSuite</a> benchmarks.
 */
#include "gemm_blocked.h"

/* ----------------- Body function imported from machsuite. ----------------- */
/*
  Implementation based on algorithm described in:
  The cache performance and optimizations of blocked algorithms
  M. D. Lam, E. E. Rothberg, and M. E. Wolf
  ASPLOS 1991
*/
void bbgemm(TYPE_GEMMB m1[N_GEMMB], TYPE_GEMMB m2[N_GEMMB], TYPE_GEMMB prod[N_GEMMB]){
  int i, k, j, jj, kk;
  int i_row, k_row;
  TYPE_GEMMB temp_x, mul;

 loopjj:for (jj = 0; jj < row_size; jj += block_size){
  loopkk:for (kk = 0; kk < row_size; kk += block_size){
    loopi:for ( i = 0; i < row_size; ++i){
      loopk:for (k = 0; k < block_size; ++k){
          i_row = i * row_size;
          k_row = (k  + kk) * row_size;
          temp_x = m1[i_row + k + kk];
        loopj:for (j = 0; j < block_size; ++j){
            mul = temp_x * m2[k_row + j + jj];
            prod[i_row + j + jj] += mul;
          }
        }
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
