/**
 * @file bfs_bulk_cls.h
 * @brief Header file for emu-hmpsoc BFS_BULK calling stubs.
 */

#ifndef _BFS_BULK_CLS
#define _BFS_BULK_CLS

#include <stdint.h>
#include <fcntl.h>

#include <bfs_bulk.h>
#include <string.h>
extern "C"{
#include "api/monAlloc.h"
#include "drivers/genIp_utils.h"
}


#define DEV_BBULK "/dev/sc_bbulk"
int bfs_bulk(void* in, void* &out, bool &localAlloc);
int bfs_bulk_hw(void* in, void* &out, bool &localAlloc);
void bfs_bulk_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);
void bfs_bulk_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);
void bfs_bulk_getDataSize(uint iter, size_t &lIn, size_t &lOut);

#endif /*_BFS_BULK_CLS*/
