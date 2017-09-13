/**
 * @file gemm_blocked_cls.h
 * @brief Header file for emu-hmpsoc GEMM_BLOCKED calling stub.
 */
#ifndef _GEMM_BLOCKED_CLS
#define _GEMM_BLOCKED_CLS

#include <gemm_blocked.h>
#include <fcntl.h>
#include <string.h>
extern "C"{
#include "api/monAlloc.h"
#include "drivers/genIp_utils.h"
}

#define DEV_GEMM_BLOCKED "/dev/sc_gemmB"

int gemm_blocked(void* in, void* &out, bool &localAlloc);
int gemm_blocked_hw(void* in, void* &out, bool &localAlloc);
void gemm_blocked_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);
void gemm_blocked_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);
void gemm_blocked_getDataSize(uint iter, size_t &lIn, size_t &lOut);
#endif /*_GEMM_BLOCKED_CLS*/
