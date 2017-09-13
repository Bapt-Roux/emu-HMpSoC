/**
 * @file spmv_crs.h
 * @brief Header file for emu-hmpsoc SPMV_CRS function.
 */
#ifndef _SPMV_CRS_IP
#define _SPMV_CRS_IP

#include <stdlib.h>
#include <stdio.h>

// These constants valid for the IEEE 494 bus interconnect matrix
#define NNZ_SPMVC 1666
#define N_SPMVC 494

#define TYPE_SPMVC double

void spmv(TYPE_SPMVC val[NNZ_SPMVC], int32_t cols[NNZ_SPMVC], int32_t rowDelimiters[N_SPMVC + 1],
          TYPE_SPMVC vec[N_SPMVC], TYPE_SPMVC out[N_SPMVC]);

#endif /*_SPMV_CRS_H*/
