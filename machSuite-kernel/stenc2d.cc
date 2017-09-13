/**
 * @file stenc2d.cc
 * @brief Implementation of emu-hmpsoc STENC2D functions.
 * Body of functions extracted from <a href=https://github.com/breagen/MachSuite> MachSuite</a> benchmarks.
 */
#include "stenc2d.h"

/* ----------------- Body function imported from machsuite. ----------------- */
void stencil (TYPE_S2D orig[ROW_S2D * COL_S2D], TYPE_S2D sol[ROW_S2D * COL_S2D], TYPE_S2D filter[f_size]){
  int r, c, k1, k2;
  TYPE_S2D temp, mul;

 stencil_label1:for (r=0; r<ROW_S2D-2; r++) {
  stencil_label2:for (c=0; c<COL_S2D-2; c++) {
      temp = (TYPE_S2D)0;
    stencil_label3:for (k1=0;k1<3;k1++){
      stencil_label4:for (k2=0;k2<3;k2++){
          mul = filter[k1*3 + k2] * orig[(r+k1)*COL_S2D + c+k2];
          temp += mul;
        }
      }
      sol[(r*COL_S2D) + c] = temp;
    }
  }

 border_cc1:for (r=0; r<ROW_S2D-2; r++) {
  border_cc2:for (c=COL_S2D-2; c<COL_S2D; c++) {
      sol[(r*COL_S2D) + c] = 0;
    }
  }
 border_cr1:for (r=ROW_S2D-2; r<ROW_S2D; r++) {
  border_cr2:for (c=0; c<COL_S2D; c++) {
      sol[(r*COL_S2D) + c] = 0;
    }
  }
}
/* -------------------------------------------------------------------------- */
