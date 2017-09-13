/**
 * @file viterbi_cls.h
 * @brief Header file for emu-hmpsoc VITERBI calling stubs.
 */

#ifndef _VITERBI_CLS
#define _VITERBI_CLS

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>

#include <viterbi.h>
#include <string.h>
extern "C"{
#include "api/monAlloc.h"
#include "drivers/genIp_utils.h"
}


#define DEV_VITERBI "/dev/sc_viterbi"
int viterbi(void* in, void* &out, bool &localAlloc);
int viterbi_hw(void* in, void* &out, bool &localAlloc);
void viterbi_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);
void viterbi_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);
void viterbi_getDataSize(uint iter, size_t &lIn, size_t &lOut);
#endif /*_VITERBI_CLS*/
