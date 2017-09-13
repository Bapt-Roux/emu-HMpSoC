/**
 * @file fft_strided.h
 * @brief Header file for emu-hmpsoc FFT_STRIDED function.
 */
#ifndef _FFT_STRIDED_IP
#define _FFT_STRIDED_IP

#include <stdio.h>
#include <stdlib.h>

#define FFT_SIZE 1024
#define twoPI 6.28318530717959

void fft(double real[FFT_SIZE], double img[FFT_SIZE], double real_twid[FFT_SIZE/2], double img_twid[FFT_SIZE/2]);
#endif /*_FFT_STRIDED_H*/
