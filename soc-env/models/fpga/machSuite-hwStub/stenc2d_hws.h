/**
 * @file stenc2d_hws.h
 * @brief Header file for emu-hmpsoc STENC2D HW stub.
 */
#ifndef _STENC2D_HWS
#define _STENC2D_HWS

#include <models/fpga/hwIP.h>
#include <stenc2d.h>

/**
 * @class stenc2d_hwStub
 * @brief integrated the stenc2d computation function within a systemC Ip.
 *----------------------------------------------------------------------------*/
class stenc2d_hwStub: public hwIP
{
  uint cTime_ns, cEn_nj;
  void parse_cmd();

  TYPE_S2D filter[f_size];
  TYPE_S2D inputs[ROW_S2D*COL_S2D];

public:
  SC_HAS_PROCESS(stenc2d_hwStub);
  stenc2d_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj);
  ~stenc2d_hwStub();
};

#endif /*_STENC2D_HWS*/
