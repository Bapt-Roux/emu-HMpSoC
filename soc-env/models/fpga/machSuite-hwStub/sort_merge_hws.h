/**
 * @file sort_merge_hws.h
 * @brief Header file for emu-hmpsoc SORT_MERGE HW stub.
 */
#ifndef _SORT_MERGE_HWS
#define _SORT_MERGE_HWS

#include <models/fpga/hwIP.h>
#include <sort_merge.h>

/**
 * @class sort_merge_hwStub
 * @brief integrated the sort_merge computation function within a systemC Ip.
 *----------------------------------------------------------------------------*/
class sort_merge_hwStub: public hwIP
{
  uint cTime_ns, cEn_nj;
  void parse_cmd();

  TYPE_SM data[SIZE];

public:
  SC_HAS_PROCESS(sort_merge_hwStub);
  sort_merge_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj);
  ~sort_merge_hwStub();
};

#endif /*_SORT_MERGE_HWS*/
