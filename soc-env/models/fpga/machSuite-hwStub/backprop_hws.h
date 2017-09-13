/**
 * @file backprop_hws.h
 * @brief Header file for emu-hmpsoc BACKPROP HW stub.
 */
#ifndef _BACKPROP_HWS
#define _BACKPROP_HWS

#include <models/fpga/hwIP.h>
#include <backprop.h>

/**
 * @class backprop_hwStub
 * @brief integrated the backprop computation function within a systemC Ip.
 *----------------------------------------------------------------------------*/
class backprop_hwStub: public hwIP
{
  uint cTime_ns, cEn_nj;
  void parse_cmd();

  TYPE_BP training_data[training_sets*input_dimension];
  TYPE_BP training_targets[training_sets*possible_outputs];
  TYPE_BP weights1[input_dimension*nodes_per_layer];
  TYPE_BP weights2[nodes_per_layer*nodes_per_layer];
  TYPE_BP weights3[nodes_per_layer*possible_outputs];
  TYPE_BP biases1[nodes_per_layer];
  TYPE_BP biases2[nodes_per_layer];
  TYPE_BP biases3[possible_outputs];

public:
  SC_HAS_PROCESS(backprop_hwStub);
  backprop_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj);
  ~backprop_hwStub();
};

#endif /*_BACKPROP_HWS*/
