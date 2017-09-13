/**
 * @file viterbi.h
 * @brief Header file for emu-hmpsoc VITERBI function.
 */

#ifndef _VITERBI_IP
#define _VITERBI_IP

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#define TYPE double
typedef uint8_t tok_t;
typedef TYPE prob_t;
typedef uint8_t state_t;
typedef int32_t step_t;

#define N_STATES  64
#define N_OBS     140
#define N_TOKENS  64
// #define N_STATES  5
// #define N_OBS     32
// #define N_TOKENS  9

int viterbi( tok_t obs[N_OBS], prob_t init[N_STATES], prob_t transition[N_STATES*N_STATES], prob_t emission[N_STATES*N_TOKENS], state_t path[N_OBS] );

#endif /*_VITERBI_IP*/
