/**
 * @file nw_hws.h
 * @brief Header file for emu-hmpsoc NW HW stub.
 */
#ifndef _NW_HWS
#define _NW_HWS

#include <models/fpga/hwIP.h>
#include <nw.h>

/**
 * @class nw_hwStub
 * @brief integrated the nw computation function within a systemC Ip.
 *----------------------------------------------------------------------------*/
class nw_hwStub: public hwIP
{
  uint cTime_ns, cEn_nj;
  void parse_cmd();

  char seqA[ALEN];
  char seqB[BLEN];
  char alignA[ALEN+BLEN];
  char alignB[ALEN+BLEN];

  int M[(ALEN+1)*(BLEN+1)];
  char ptr[(ALEN+1)*(BLEN+1)];

public:
  SC_HAS_PROCESS(nw_hwStub);
  nw_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj);
  ~nw_hwStub();
};

#endif /*_NW_HWS*/
