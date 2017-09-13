/**
 * @file fft_strided_hws.h
 * @brief Header file for emu-hmpsoc FFT_STRIDED HW stub.
 */
#ifndef _FFT_STRIDED_HWS
#define _FFT_STRIDED_HWS

#include <models/fpga/hwIP.h>
#include <fft_strided.h>

/**
 * @class fft_strided_hwStub
 * @brief integrated the fft_strided computation function within a systemC Ip.
 *----------------------------------------------------------------------------*/
class fft_strided_hwStub: public hwIP
{
  uint cTime_ns, cEn_nj;
  void parse_cmd();

  double tw_real[FFT_SIZE/2];
  double tw_img[FFT_SIZE/2];
  double s_real[FFT_SIZE];
  double s_img[FFT_SIZE];

public:
  SC_HAS_PROCESS(fft_strided_hwStub);
  fft_strided_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj);
  ~fft_strided_hwStub();
};

#endif /*_FFT_STRIDED_HWS*/
