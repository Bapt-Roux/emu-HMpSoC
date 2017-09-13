/**
 * @file fft_strided_cls.h
 * @brief Header file for emu-hmpsoc FFT_STRIDED calling stub.
 */
#ifndef _FFT_STRIDED_CLS
#define _FFT_STRIDED_CLS

#include <fft_strided.h>
#include <fcntl.h>
#include <string.h>
extern "C"{
#include "api/monAlloc.h"
#include "drivers/genIp_utils.h"
}

#define DEV_FFT_STRIDED "/dev/sc_fftS"

int fft_strided(void* in, void* &out, bool &localAlloc);
int fft_strided_hw(void* in, void* &out, bool &localAlloc);
void fft_strided_genData(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);
void fft_strided_getSamples(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);
void fft_strided_getDataSize(uint iter, size_t &lIn, size_t &lOut);
#endif /*_FFT_STRIDED_CLS*/
