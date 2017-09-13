/**
 * @file stenc2d.h
 * @brief Header file for emu-hmpsoc STENC2D function.
 */
#ifndef _STENC2D_IP
#define _STENC2D_IP
#include <stdio.h>
#include <stdlib.h>

//Define input sizes
#define COL_S2D 64
#define ROW_S2D 128
#define f_size 9

//Data Bounds
#define TYPE_S2D int32_t

void stencil( TYPE_S2D orig[ROW_S2D * COL_S2D],
              TYPE_S2D sol[ROW_S2D * COL_S2D],
              TYPE_S2D filter[f_size] );

#endif /*_STENC2D_H*/
