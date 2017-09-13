/**
 * @file aes.h
 * @brief Header file for emu-hmpsoc AES function.
 */
#ifndef _AES_IP
#define _AES_IP

#include <stdint.h>
/** @structure Retain the internal state of aes execution */
typedef struct {
  uint8_t key[32];
  uint8_t enckey[32];
  uint8_t deckey[32];
} aes256_context;

void aes256_encrypt_ecb(aes256_context *ctx, uint8_t k[32], uint8_t buf[16]);

#endif /*_AES_H*/
