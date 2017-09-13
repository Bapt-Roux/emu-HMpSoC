/**
 * @file kmp_cls.h
 * @brief Header file for emu-hmpsoc KMP calling stub.
 */
#ifndef _KMP_CLS
#define _KMP_CLS

#include <kmp.h>
#include <fcntl.h>
#include <string.h>
extern "C"{
#include "api/monAlloc.h"
#include "drivers/genIp_utils.h"
}

#define DEV_KMP "/dev/sc_kmp"

int kmp(void* in, void* &out, bool &localAlloc);
int kmp_hw(void* in, void* &out, bool &localAlloc);
void kmp_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);
void kmp_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);
void kmp_getDataSize(uint iter, size_t &lIn, size_t &lOut);
#endif /*_KMP_CLS*/
