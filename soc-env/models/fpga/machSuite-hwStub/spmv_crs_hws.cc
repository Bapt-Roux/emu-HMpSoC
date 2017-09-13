/**
 * @file spmv_crs_hws.cc
 * @brief HW Stub for SPMV_CRS from machsuite.
 */
#include "spmv_crs_hws.h"

/**
 * @brief emu-hmpsoc machsuite spmv_crs HW_Stub constructor.
 * @params name: systemC entity name.
 * @params compTime_ns: execution Time[ns] of one iteration.
 * @params compTime_ns: execution Energy[nJ] of one iteration.
 */
spmv_crs_hwStub::spmv_crs_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj)
  : hwIP(name, (uint8_t)1 /*One HPx*/, (uint8_t)1 /*One irq*/, GENIP_RPSIZE),
    cTime_ns(compTime_ns), cEn_nj(compEn_nj)
{
  //Register THREAD
  SC_THREAD(parse_cmd);
  sensitive << hwIP::cmd_events;
}

spmv_crs_hwStub::~spmv_crs_hwStub(){
}

/**
 * @brief emu-hmpsoc SC_Thread implementation for machsuite spmv_crs HW_Stub .
 */
void spmv_crs_hwStub::parse_cmd(){
  wait(); // prevent execution at bootup
  while(1) {
#ifdef MACHSUITE_DEBUG
    std::cout << "@ "<< sc_time_stamp() << " ==>"<< "SPMV_CRS received cmd." << std::endl;
#endif
    switch (*((uint8_t*)_REG(CMD)))
      {
      case CMD_START:
        irqn[0].write(0); // reset irq Done
        *((uint8_t*)_REG(STATUS)) &= ~(STATUS_DONE + STATUS_IRQ(0));
#ifdef MACHSUITE_DEBUG
        std::cout << "SPMV_CRS_HW started for "<< (uint32_t) *((uint32_t*)_REG(PARAMS)) <<" iterations\n";
        std::cout << "ARRAY IN 0x"<<std::hex << (uintptr_t) *((uint32_t*)_REG(ARRAY_IN)) <<"\n";
        std::cout << "ARRAY OUT 0x"<<std::hex << (uintptr_t) *((uint32_t*)_REG(ARRAY_OUT)) <<"\n";
#endif
        for( uint i=0; i< *((uint32_t*)_REG(PARAMS)); i++){
          //read input data
          memRead(*((uint32_t*)_REG(ARRAY_IN))
                  + i*(sizeof(TYPE_SPMVC)* (NNZ_SPMVC +N_SPMVC) + sizeof(int32_t)*(N_SPMVC+NNZ_SPMVC+1)),
                  val, sizeof(TYPE_SPMVC)*(NNZ_SPMVC), 0);
          memRead(*((uint32_t*)_REG(ARRAY_IN)) +sizeof(TYPE_SPMVC)*(NNZ_SPMVC)
                  + i*(sizeof(TYPE_SPMVC)* (NNZ_SPMVC +N_SPMVC) + sizeof(int32_t)*(N_SPMVC+NNZ_SPMVC+1)),
                  cols, sizeof(int32_t)*(NNZ_SPMVC), 0);
          memRead(*((uint32_t*)_REG(ARRAY_IN)) +(sizeof(TYPE_SPMVC)+sizeof(int32_t))*(NNZ_SPMVC)
                  + i*(sizeof(TYPE_SPMVC)* (NNZ_SPMVC +N_SPMVC) + sizeof(int32_t)*(N_SPMVC+NNZ_SPMVC+1)),
                  rowDelimiters, sizeof(int32_t)*(N_SPMVC+1), 0);
          memRead(*((uint32_t*)_REG(ARRAY_IN))
                  + (sizeof(TYPE_SPMVC)* (NNZ_SPMVC) + sizeof(int32_t)*(N_SPMVC+NNZ_SPMVC+1))
                  + i*(sizeof(TYPE_SPMVC)* (NNZ_SPMVC +N_SPMVC) + sizeof(int32_t)*(N_SPMVC+NNZ_SPMVC+1)),
                  vec, sizeof(TYPE_SPMVC)*(N_SPMVC), 0);
          spmv(val, cols, rowDelimiters, vec, out);
          wait(cTime_ns, SC_NS);
          // FIXME: add Computation Energy monitoring feature
          // FIXME: append energy and time cost to monSystem
          //write output
          memWrite(*((uint32_t*)_REG(ARRAY_OUT))
                   + i*(sizeof(TYPE_SPMVC)*(N_SPMVC)), out, sizeof(TYPE_SPMVC)*N_SPMVC, 0);
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
