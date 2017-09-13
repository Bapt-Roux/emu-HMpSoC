/**
 * @file aes_hws.h
 * @brief Header file for emu-hmpsoc AES HW stub.
 */
#ifndef _AES_HWS
#define _AES_HWS

#include <models/fpga/hwIP.h>
#include <aes.h>

/**
 * @class aes_hwStub
 * @brief integrated the aes256 computation function within a systemC Ip.
 *----------------------------------------------------------------------------*/
class aes_hwStub: public hwIP
{
  uint cTime_ns, cEn_nj;
  aes256_context ctx;
  uint8_t key[32];
  uint8_t buf[16];
  void parse_cmd();

public:
  SC_HAS_PROCESS(aes_hwStub);
  aes_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj);
  ~aes_hwStub();
};

#endif /*_AES_HWS*/
