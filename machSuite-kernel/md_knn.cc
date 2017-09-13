/**
 * @file md_knn.cc
 * @brief Implementation of emu-hmpsoc MD_KNN functions.
 * Body of functions extracted from <a href=https://github.com/breagen/MachSuite> MachSuite</a> benchmarks.
 */
#include "md_knn.h"

/* ----------------- Body function imported from machsuite. ----------------- */
/*
Implemenataion based on:
A. Danalis, G. Marin, C. McCurdy, J. S. Meredith, P. C. Roth, K. Spafford, V. Tipparaju, and J. S. Vetter.
The scalable heterogeneous computing (shoc) benchmark suite.
In Proceedings of the 3rd Workshop on General-Purpose Computation on Graphics Processing Units, 2010.
*/

void md_kernel(TYPE_MDK force_x[nAtoms],
               TYPE_MDK force_y[nAtoms],
               TYPE_MDK force_z[nAtoms],
               TYPE_MDK position_x[nAtoms],
               TYPE_MDK position_y[nAtoms],
               TYPE_MDK position_z[nAtoms],
               int32_t NL[nAtoms*maxNeighbors])
{
    TYPE_MDK delx, dely, delz, r2inv;
    TYPE_MDK r6inv, potential, force, j_x, j_y, j_z;
    TYPE_MDK i_x, i_y, i_z, fx, fy, fz;

    int32_t i, j, jidx;

loop_i : for (i = 0; i < nAtoms; i++){
             i_x = position_x[i];
             i_y = position_y[i];
             i_z = position_z[i];
             fx = 0;
             fy = 0;
             fz = 0;
loop_j : for( j = 0; j < maxNeighbors; j++){
             // Get neighbor
             jidx = NL[i*maxNeighbors + j];
             // Look up x,y,z positions
             j_x = position_x[jidx];
             j_y = position_y[jidx];
             j_z = position_z[jidx];
             // Calc distance
             delx = i_x - j_x;
             dely = i_y - j_y;
             delz = i_z - j_z;
             r2inv = 1.0/( delx*delx + dely*dely + delz*delz );
             // Assume no cutoff and aways account for all nodes in area
             r6inv = r2inv * r2inv * r2inv;
             potential = r6inv*(lj1*r6inv - lj2);
             // Sum changes in force
             force = r2inv*potential;
             fx += delx * force;
             fy += dely * force;
             fz += delz * force;
         }
         //Update forces after all neighbors accounted for.
         force_x[i] = fx;
         force_y[i] = fy;
         force_z[i] = fz;
         //printf("dF=%lf,%lf,%lf\n", fx, fy, fz);
         }
}
/* -------------------------------------------------------------------------- */
