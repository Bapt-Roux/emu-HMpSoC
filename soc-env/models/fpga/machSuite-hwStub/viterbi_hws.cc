/**
 * @file viterbi_hws.cc
 * @brief HW Stub for VITERBI from machsuite.
 */
#include "viterbi_hws.h"

/**
 * @brief emu-hmpsoc machsuite viterbi HW_Stub constructor.
 * @params name: systemC entity name.
 * @params compTime_ns: execution Time[ns] of one iteration.
 * @params compTime_ns: execution Energy[nJ] of one iteration.
 */
viterbi_hwStub::viterbi_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj)
  : hwIP(name, (uint8_t)1 /*One HPx*/, (uint8_t)1 /*One irq*/, GENIP_RPSIZE),
    cTime_ns(compTime_ns), cEn_nj(compEn_nj)
{
  //Register THREAD
  SC_THREAD(parse_cmd);
  sensitive << hwIP::cmd_events;
}

viterbi_hwStub::~viterbi_hwStub(){
}

/**
 * @brief emu-hmpsoc SC_Thread implementation for machsuite viterbi HW_Stub .
 */
void viterbi_hwStub::parse_cmd(){
  wait(); // prevent execution at bootup
  while(1) {
#ifdef MACHSUITE_DEBUG
    std::cout << "@ "<< sc_time_stamp() << " ==>"<< "VITERBI received cmd." << std::endl;
#endif
    switch (*((uint8_t*)_REG(CMD)))
      {
      case CMD_START:
        irqn[0].write(0); // reset irq Done
        *((uint8_t*)_REG(STATUS)) &= ~(STATUS_DONE + STATUS_IRQ(0));
#ifdef MACHSUITE_DEBUG
        std::cout << "VITERBI_HW started for "<< (uint32_t) *((uint32_t*)_REG(PARAMS)) <<" iterations\n";
        std::cout << "ARRAY IN 0x"<<std::hex << (uintptr_t) *((uint32_t*)_REG(ARRAY_IN)) <<"\n";
        std::cout << "ARRAY OUT 0x"<<std::hex << (uintptr_t) *((uint32_t*)_REG(ARRAY_OUT)) <<"\n";
#endif
        for( uint i=0; i< *((uint32_t*)_REG(PARAMS)); i++){
          //read input data
          memRead(*((uint32_t*)_REG(ARRAY_IN))
                  + i*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS))),
                  obs, N_OBS*sizeof(tok_t), 0);

          memRead(*((uint32_t*)_REG(ARRAY_IN))
                  + i*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)))
                  + sizeof(tok_t)*N_OBS,
                  init, N_STATES*sizeof(prob_t), 0);
          memRead(*((uint32_t*)_REG(ARRAY_IN))
                  + i*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)))
                  + (sizeof(tok_t)*N_OBS + sizeof(prob_t)*N_STATES),
                  transition, (N_STATES*N_STATES)*sizeof(prob_t), 0);
          memRead(*((uint32_t*)_REG(ARRAY_IN))
                  + i*(sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES + N_TOKENS)))
                  + (sizeof(tok_t)*N_OBS + sizeof(prob_t)*(N_STATES*(1 + N_STATES ))),
                  emission, (N_STATES*N_TOKENS)*sizeof(prob_t), 0);

          viterbi(obs, init, transition, emission, path);
          wait(cTime_ns, SC_NS);
          // FIXME: add Computation Energy monitoring feature
          // FIXME: append energy and time cost to monSystem
          //write output
          memWrite(*((uint32_t*)_REG(ARRAY_OUT)) + i*N_OBS*sizeof(state_t),
                  path, N_OBS*sizeof(state_t), 0);
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
