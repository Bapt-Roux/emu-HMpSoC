/**
 * @file nw_cls.h
 * @brief Header file for emu-hmpsoc NW calling stub.
 */
#ifndef _NW_CLS
#define _NW_CLS

#include <nw.h>
#include <fcntl.h>
#include <string.h>
extern "C"{
#include "api/monAlloc.h"
#include "drivers/genIp_utils.h"
}

#define DEV_NW "/dev/sc_nw"

int nw(void* in, void* &out, bool &localAlloc);
int nw_hw(void* in, void* &out, bool &localAlloc);
void nw_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);
void nw_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);
void nw_getDataSize(uint iter, size_t &lIn, size_t &lOut);
#endif /*_NW_CLS*/
