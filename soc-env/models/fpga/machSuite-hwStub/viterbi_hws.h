/**
 * @file viterbi_hws.h
 * @brief Header file for emu-hmpsoc VITERBI HW stub.
 */
#ifndef _VITERBI_HWS
#define _VITERBI_HWS

#include <models/fpga/hwIP.h>
#include <viterbi.h>

/**
 * @class viterbi_hwStub
 * @brief integrated the viterbi computation function within a systemC Ip.
 *----------------------------------------------------------------------------*/
class viterbi_hwStub: public hwIP
{
  uint cTime_ns, cEn_nj;
  void parse_cmd();
  tok_t obs[N_OBS];
  prob_t init[N_STATES], transition[N_STATES*N_STATES], emission[N_STATES*N_TOKENS];
  state_t path[N_OBS];

public:
  SC_HAS_PROCESS(viterbi_hwStub);
  viterbi_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj);
  ~viterbi_hwStub();
};

#endif /*_AES_HWS*/
