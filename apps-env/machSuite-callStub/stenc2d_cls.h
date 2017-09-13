/**
 * @file stenc2d_cls.h
 * @brief Header file for emu-hmpsoc STENC2D calling stub.
 */
#ifndef _STENC2D_CLS
#define _STENC2D_CLS

#include <stenc2d.h>
#include <fcntl.h>
#include <string.h>
extern "C"{
#include "api/monAlloc.h"
#include "drivers/genIp_utils.h"
}

#define DEV_STENC2D "/dev/sc_stenc2d"

int stenc2d(void* in, void* &out, bool &localAlloc);
int stenc2d_hw(void* in, void* &out, bool &localAlloc);
void stenc2d_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);
void stenc2d_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);
void stenc2d_getDataSize(uint iter, size_t &lIn, size_t &lOut);
#endif /*_STENC2D_CLS*/
