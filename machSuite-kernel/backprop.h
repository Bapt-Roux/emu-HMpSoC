/**
 * @file backprop.h
 * @brief Header file for emu-hmpsoc BACKPROP function.
 */
#ifndef _BACKPROP_IP
#define _BACKPROP_IP
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Fixed parameters
#define input_dimension  13
#define possible_outputs  3
#define training_sets   163
#define nodes_per_layer  64
#define layers            2
#define learning_rate  0.01
#define epochs            1
#define test_sets        15
#define norm_param    0.005

//Data Bounds
#define TYPE_BP double

void backprop(
              TYPE_BP weights1[input_dimension*nodes_per_layer],
              TYPE_BP weights2[nodes_per_layer*nodes_per_layer],
              TYPE_BP weights3[nodes_per_layer*possible_outputs],
              TYPE_BP biases1[nodes_per_layer],
              TYPE_BP biases2[nodes_per_layer],
              TYPE_BP biases3[possible_outputs],
              TYPE_BP training_data[training_sets*input_dimension],
              TYPE_BP training_targets[training_sets*possible_outputs]);

#endif /*_BACKPROP_H*/
