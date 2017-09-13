/**
 * @file gemm_blocked_hws.h
 * @brief Header file for emu-hmpsoc GEMM_BLOCKED HW stub.
 */
#ifndef _GEMM_BLOCKED_HWS
#define _GEMM_BLOCKED_HWS

#include <models/fpga/hwIP.h>
#include <gemm_blocked.h>

/**
 * @class gemm_blocked_hwStub
 * @brief integrated the gemm_blocked computation function within a systemC Ip.
 *----------------------------------------------------------------------------*/
class gemm_blocked_hwStub: public hwIP
{
  uint cTime_ns, cEn_nj;
  void parse_cmd();

  TYPE_GEMMB m1[N_GEMMB];
  TYPE_GEMMB m2[N_GEMMB];
  TYPE_GEMMB prod[N_GEMMB];

public:
  SC_HAS_PROCESS(gemm_blocked_hwStub);
  gemm_blocked_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj);
  ~gemm_blocked_hwStub();
};

#endif /*_GEMM_BLOCKED_HWS*/
