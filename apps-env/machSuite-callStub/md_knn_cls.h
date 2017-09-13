/**
 * @file md_knn_cls.h
 * @brief Header file for emu-hmpsoc MD_KNN calling stub.
 */
#ifndef _MD_KNN_CLS
#define _MD_KNN_CLS

#include <md_knn.h>
#include <fcntl.h>
#include <string.h>
extern "C"{
#include "api/monAlloc.h"
#include "drivers/genIp_utils.h"
}

#define DEV_MD_KNN "/dev/sc_mdKnn"

int md_knn(void* in, void* &out, bool &localAlloc);
int md_knn_hw(void* in, void* &out, bool &localAlloc);
void md_knn_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);
void md_knn_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);
void md_knn_getDataSize(uint iter, size_t &lIn, size_t &lOut);
#endif /*_MD_KNN_CLS*/
