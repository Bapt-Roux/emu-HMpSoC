/**
 * @file sort_merge_cls.h
 * @brief Header file for emu-hmpsoc SORT_MERGE calling stub.
 */
#ifndef _SORT_MERGE_CLS
#define _SORT_MERGE_CLS

#include <sort_merge.h>
#include <fcntl.h>
#include <string.h>
extern "C"{
#include "api/monAlloc.h"
#include "drivers/genIp_utils.h"
}

#define DEV_SORT_MERGE "/dev/sc_sortM"

int sort_merge(void* in, void* &out, bool &localAlloc);
int sort_merge_hw(void* in, void* &out, bool &localAlloc);
void sort_merge_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);
void sort_merge_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);
void sort_merge_getDataSize(uint iter, size_t &lIn, size_t &lOut);
#endif /*_SORT_MERGE_CLS*/
