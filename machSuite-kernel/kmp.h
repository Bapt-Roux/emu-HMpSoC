/**
 * @file kmp.h
 * @brief Header file for emu-hmpsoc KMP function.
 */
#ifndef _KMP_IP
#define _KMP_IP
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#define PATTERN_SIZE 4
#define STRING_SIZE (32411)

int kmp(char pattern[PATTERN_SIZE], char input[STRING_SIZE], int32_t kmpNext[PATTERN_SIZE], int32_t n_matches[1]);
#endif /*_KMP_H*/
