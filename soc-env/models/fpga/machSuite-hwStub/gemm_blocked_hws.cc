/**
 * @file gemm_blocked_hws.cc
 * @brief HW Stub for GEMM_BLOCKED from machsuite.
 */
#include "gemm_blocked_hws.h"

/**
 * @brief emu-hmpsoc machsuite gemm_blocked HW_Stub constructor.
 * @params name: systemC entity name.
 * @params compTime_ns: execution Time[ns] of one iteration.
 * @params compTime_ns: execution Energy[nJ] of one iteration.
 */
gemm_blocked_hwStub::gemm_blocked_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj)
  : hwIP(name, (uint8_t)1 /*One HPx*/, (uint8_t)1 /*One irq*/, GENIP_RPSIZE),
    cTime_ns(compTime_ns), cEn_nj(compEn_nj)
{
  //Register THREAD
  SC_THREAD(parse_cmd);
  sensitive << hwIP::cmd_events;
}

gemm_blocked_hwStub::~gemm_blocked_hwStub(){
}

/**
 * @brief emu-hmpsoc SC_Thread implementation for machsuite gemm_blocked HW_Stub .
 */
void gemm_blocked_hwStub::parse_cmd(){
  wait(); // prevent execution at bootup
  while(1) {
#ifdef MACHSUITE_DEBUG
    std::cout << "@ "<< sc_time_stamp() << " ==>"<< "GEMM_BLOCKED received cmd." << std::endl;
#endif
    switch (*((uint8_t*)_REG(CMD)))
      {
      case CMD_START:
        irqn[0].write(0); // reset irq Done
        *((uint8_t*)_REG(STATUS)) &= ~(STATUS_DONE + STATUS_IRQ(0));
#ifdef MACHSUITE_DEBUG
        std::cout << "GEMM_BLOCKED_HW started for "<< (uint32_t) *((uint32_t*)_REG(PARAMS)) <<" iterations\n";
        std::cout << "ARRAY IN 0x"<<std::hex << (uintptr_t) *((uint32_t*)_REG(ARRAY_IN)) <<"\n";
        std::cout << "ARRAY OUT 0x"<<std::hex << (uintptr_t) *((uint32_t*)_REG(ARRAY_OUT)) <<"\n";
#endif
        for( uint i=0; i< *((uint32_t*)_REG(PARAMS)); i++){
          //read input data
          memRead(*((uint32_t*)_REG(ARRAY_IN))
                  + i*(sizeof(TYPE_GEMMB)* (2*N_GEMMB)), m1, sizeof(TYPE_GEMMB)*(N_GEMMB), 0);
          memRead(*((uint32_t*)_REG(ARRAY_IN)) + sizeof(TYPE_GEMMB)*N_GEMMB
                  + i*(sizeof(TYPE_GEMMB)* (2*N_GEMMB)), m2, sizeof(TYPE_GEMMB)*(N_GEMMB), 0);
          for (uint k=0; k<N_GEMMB; k++){ *((TYPE_GEMMB*)prod +k)=0.0;}

          bbgemm(m1, m2, prod);
          wait(cTime_ns, SC_NS);
          // FIXME: add Computation Energy monitoring feature
          // FIXME: append energy and time cost to monSystem
          //write output
          memWrite(*((uint32_t*)_REG(ARRAY_OUT))
                   + i*(sizeof(TYPE_GEMMB)*(N_GEMMB)), prod, sizeof(TYPE_GEMMB)*N_GEMMB, 0);
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
