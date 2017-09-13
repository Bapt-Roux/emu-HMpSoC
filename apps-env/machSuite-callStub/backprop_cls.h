/**
 * @file backprop_cls.h
 * @brief Header file for emu-hmpsoc BACKPROP calling stub.
 */
#ifndef _BACKPROP_CLS
#define _BACKPROP_CLS

#include <backprop.h>
#include <fcntl.h>
#include <string.h>
extern "C"{
#include "api/monAlloc.h"
#include "drivers/genIp_utils.h"
}

#define DEV_BACKPROP "/dev/sc_backprop"

int backprop(void* in, void* &out, bool &localAlloc);
int backprop_hw(void* in, void* &out, bool &localAlloc);
void backprop_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);
void backprop_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);
void backprop_getDataSize(uint iter, size_t &lIn, size_t &lOut);
#endif /*_BACKPROP_CLS*/
