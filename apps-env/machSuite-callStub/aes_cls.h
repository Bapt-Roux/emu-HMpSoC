/**
 * @file aes_cls.h
 * @brief Header file for emu-hmpsoc AES calling stub.
 */
#ifndef _AES_CLS
#define _AES_CLS

#include <aes.h>
#include <fcntl.h>
#include <string.h>
extern "C"{
#include "api/monAlloc.h"
#include "drivers/genIp_utils.h"
}

#define DEV_AES "/dev/sc_aes"

int aes(void* in, void* &out, bool &localAlloc);
int aes_hw(void* in, void* &out, bool &localAlloc);
void aes_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);
void aes_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);
void aes_getDataSize(uint iter, size_t &lIn, size_t &lOut);
#endif /*_AES_CLS*/
