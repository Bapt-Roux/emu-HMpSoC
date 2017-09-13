/**
 * @file kmp_hws.h
 * @brief Header file for emu-hmpsoc KMP HW stub.
 */
#ifndef _KMP_HWS
#define _KMP_HWS

#include <models/fpga/hwIP.h>
#include <kmp.h>

/**
 * @class kmp_hwStub
 * @brief integrated the kmp computation function within a systemC Ip.
 *----------------------------------------------------------------------------*/
class kmp_hwStub: public hwIP
{
  uint cTime_ns, cEn_nj;
  void parse_cmd();

  char pattern[PATTERN_SIZE];
  char text[STRING_SIZE];
  int32_t kmpNext[PATTERN_SIZE];
  int32_t match;

public:
  SC_HAS_PROCESS(kmp_hwStub);
  kmp_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj);
  ~kmp_hwStub();
};

#endif /*_KMP_HWS*/
