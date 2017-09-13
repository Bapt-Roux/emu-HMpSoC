/**
 * @file md_knn.h
 * @brief Header file for emu-hmpsoc MD_KNN function.
 */
#ifndef _MD_KNN_IP
#define _MD_KNN_IP
#include <stdlib.h>
#include <stdio.h>

#define TYPE_MDK double

// Problem Constants
#define nAtoms        256
#define maxNeighbors  16
// LJ coefficients
#define lj1           1.5
#define lj2           2.0

void md_kernel(TYPE_MDK force_x[nAtoms],
               TYPE_MDK force_y[nAtoms],
               TYPE_MDK force_z[nAtoms],
               TYPE_MDK position_x[nAtoms],
               TYPE_MDK position_y[nAtoms],
               TYPE_MDK position_z[nAtoms],
               int32_t NL[nAtoms*maxNeighbors]);
#endif /*_MD_KNN_H*/
