/**
 * @file spmv_crs_cls.h
 * @brief Header file for emu-hmpsoc SPMV_CRS calling stub.
 */
#ifndef _SPMV_CRS_CLS
#define _SPMV_CRS_CLS

#include <spmv_crs.h>
#include <fcntl.h>
#include <string.h>
extern "C"{
#include "api/monAlloc.h"
#include "drivers/genIp_utils.h"
}

#define DEV_SPMV_CRS "/dev/sc_spmvC"

int spmv_crs(void* in, void* &out, bool &localAlloc);
int spmv_crs_hw(void* in, void* &out, bool &localAlloc);
void spmv_crs_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);
void spmv_crs_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);
void spmv_crs_getDataSize(uint iter, size_t &lIn, size_t &lOut);
#endif /*_SPMV_CRS_CLS*/
