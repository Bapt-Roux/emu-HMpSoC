/**
 * @file fft_strided_hws.cc
 * @brief HW Stub for FFT_STRIDED from machsuite.
 */
#include "fft_strided_hws.h"

/**
 * @brief emu-hmpsoc machsuite fft_strided HW_Stub constructor.
 * @params name: systemC entity name.
 * @params compTime_ns: execution Time[ns] of one iteration.
 * @params compTime_ns: execution Energy[nJ] of one iteration.
 */
fft_strided_hwStub::fft_strided_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj)
  : hwIP(name, (uint8_t)1 /*One HPx*/, (uint8_t)1 /*One irq*/, GENIP_RPSIZE),
    cTime_ns(compTime_ns), cEn_nj(compEn_nj)
{
  //Register THREAD
  SC_THREAD(parse_cmd);
  sensitive << hwIP::cmd_events;
}

fft_strided_hwStub::~fft_strided_hwStub(){
}

/**
 * @brief emu-hmpsoc SC_Thread implementation for machsuite fft_strided HW_Stub .
 */
void fft_strided_hwStub::parse_cmd(){
  wait(); // prevent execution at bootup
  while(1) {
#ifdef MACHSUITE_DEBUG
    std::cout << "@ "<< sc_time_stamp() << " ==>"<< "FFT_STRIDED received cmd." << std::endl;
#endif
    switch (*((uint8_t*)_REG(CMD)))
      {
      case CMD_START:
        irqn[0].write(0); // reset irq Done
        *((uint8_t*)_REG(STATUS)) &= ~(STATUS_DONE + STATUS_IRQ(0));
#ifdef MACHSUITE_DEBUG
        std::cout << "FFT_STRIDED_HW started for "<< (uint32_t) *((uint32_t*)_REG(PARAMS)) <<" iterations\n";
        std::cout << "ARRAY IN 0x"<<std::hex << (uintptr_t) *((uint32_t*)_REG(ARRAY_IN)) <<"\n";
        std::cout << "ARRAY OUT 0x"<<std::hex << (uintptr_t) *((uint32_t*)_REG(ARRAY_OUT)) <<"\n";
#endif
        for( uint i=0; i< *((uint32_t*)_REG(PARAMS)); i++){
          //read input data
          memRead(*((uint32_t*)_REG(ARRAY_IN))
                  + i*(sizeof(double)* FFT_SIZE),
                  tw_real, sizeof(double)*(FFT_SIZE/2), 0);
          memRead(*((uint32_t*)_REG(ARRAY_IN)) + sizeof(double)*(FFT_SIZE/2)
                  + i*(sizeof(double)* FFT_SIZE),
                  tw_img, sizeof(double)*(FFT_SIZE/2), 0);

          memRead(*((uint32_t*)_REG(ARRAY_IN))
                  + *((uint32_t*)_REG(PARAMS)) *(sizeof(double)*FFT_SIZE)
                  + i*(sizeof(double)*(2*FFT_SIZE)), s_real, sizeof(double)*FFT_SIZE, 0);
          memRead(*((uint32_t*)_REG(ARRAY_IN)) + sizeof(double)*FFT_SIZE
                  + *((uint32_t*)_REG(PARAMS)) *(sizeof(double)*FFT_SIZE)
                  + i*(sizeof(double)*(2*FFT_SIZE)), s_img, sizeof(double)*FFT_SIZE, 0);

          fft(s_real, s_img, tw_real, tw_img);
          wait(cTime_ns, SC_NS);
          // FIXME: add Computation Energy monitoring feature
          // FIXME: append energy and time cost to monSystem
          //write output
          memWrite(*((uint32_t*)_REG(ARRAY_OUT))
                   + i*(sizeof(double)*(2*FFT_SIZE)), s_real, sizeof(double)*FFT_SIZE, 0);
          memWrite(*((uint32_t*)_REG(ARRAY_OUT)) + sizeof(double)*FFT_SIZE
                   + i*(sizeof(double)*(2*FFT_SIZE)), s_img, sizeof(double)*FFT_SIZE, 0);
        }
        irqn[0].write(1); // set irq Done
        *((uint8_t*)_REG(STATUS)) = STATUS_DONE + STATUS_IRQ(0);
        *((uint8_t*)_REG(CMD)) = 0x00;
        break;;

      case CMD_RST_IRQ(0):
        irqn[0].write(0); // reset irq Done
        *((uint8_t*)_REG(STATUS)) &= ~(STATUS_DONE + STATUS_IRQ(0));
        *((uint8_t*)_REG(CMD)) = 0x00;
        break;;
      default:
        *((uint8_t*)_REG(CMD)) = 0x00;
        break;;
      }
    wait();
  }
}
