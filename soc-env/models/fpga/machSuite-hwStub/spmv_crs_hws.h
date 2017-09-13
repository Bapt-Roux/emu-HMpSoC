/**
 * @file spmv_crs_hws.h
 * @brief Header file for emu-hmpsoc SPMV_CRS HW stub.
 */
#ifndef _SPMV_CRS_HWS
#define _SPMV_CRS_HWS

#include <models/fpga/hwIP.h>
#include <spmv_crs.h>

/**
 * @class spmv_crs_hwStub
 * @brief integrated the spmv_crs computation function within a systemC Ip.
 *----------------------------------------------------------------------------*/
class spmv_crs_hwStub: public hwIP
{
  uint cTime_ns, cEn_nj;
  void parse_cmd();

  TYPE_SPMVC val[NNZ_SPMVC];
  int32_t cols[NNZ_SPMVC];
  int32_t rowDelimiters[N_SPMVC+1];
  TYPE_SPMVC vec[N_SPMVC];
  TYPE_SPMVC out[N_SPMVC];

public:
  SC_HAS_PROCESS(spmv_crs_hwStub);
  spmv_crs_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj);
  ~spmv_crs_hwStub();
};

#endif /*_SPMV_CRS_HWS*/
