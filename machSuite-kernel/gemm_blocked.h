/**
 * @file gemm_blocked.h
 * @brief Header file for emu-hmpsoc GEMM_BLOCKED function.
 */
#ifndef _GEMM_BLOCKED_IP
#define _GEMM_BLOCKED_IP

#include <stdio.h>
#include <stdlib.h>

//Data Type
#define TYPE_GEMMB double

//Algorithm Parameters
#define row_size 64
#define col_size 64
#define N_GEMMB row_size*col_size
#define block_size 8
#define NUMOFBLOCKS N_GEMMB/block_size/block_size

//Define the input range to operate over
#define MIN_R 0.
#define MAX_R 1.0

void bbgemm(TYPE_GEMMB m1[N_GEMMB], TYPE_GEMMB m2[N_GEMMB], TYPE_GEMMB prod[N_GEMMB]);
#endif /*_GEMM_BLOCKED_H*/
