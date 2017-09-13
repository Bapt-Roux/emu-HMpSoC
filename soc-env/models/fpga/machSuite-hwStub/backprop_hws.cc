/**
 * @file backprop_hws.cc
 * @brief HW Stub for BACKPROP from machsuite.
 */
#include "backprop_hws.h"

/**
 * @brief emu-hmpsoc machsuite backprop HW_Stub constructor.
 * @params name: systemC entity name.
 * @params compTime_ns: execution Time[ns] of one iteration.
 * @params compTime_ns: execution Energy[nJ] of one iteration.
 */
backprop_hwStub::backprop_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj)
  : hwIP(name, (uint8_t)1 /*One HPx*/, (uint8_t)1 /*One irq*/, GENIP_RPSIZE),
    cTime_ns(compTime_ns), cEn_nj(compEn_nj)
{
  //Register THREAD
  SC_THREAD(parse_cmd);
  sensitive << hwIP::cmd_events;
}

backprop_hwStub::~backprop_hwStub(){
}

/**
 * @brief emu-hmpsoc SC_Thread implementation for machsuite backprop HW_Stub .
 */
void backprop_hwStub::parse_cmd(){
  wait(); // prevent execution at bootup
  while(1) {
#ifdef MACHSUITE_DEBUG
    std::cout << "@ "<< sc_time_stamp() << " ==>"<< "BACKPROP received cmd." << std::endl;
#endif
    switch (*((uint8_t*)_REG(CMD)))
      {
      case CMD_START:
        irqn[0].write(0); // reset irq Done
        *((uint8_t*)_REG(STATUS)) &= ~(STATUS_DONE + STATUS_IRQ(0));
#ifdef MACHSUITE_DEBUG
        std::cout << "BACKPROP_HW started for "<< (uint32_t) *((uint32_t*)_REG(PARAMS)) <<" iterations\n";
        std::cout << "ARRAY IN 0x"<<std::hex << (uintptr_t) *((uint32_t*)_REG(ARRAY_IN)) <<"\n";
        std::cout << "ARRAY OUT 0x"<<std::hex << (uintptr_t) *((uint32_t*)_REG(ARRAY_OUT)) <<"\n";
#endif
        for( uint i=0; i< *((uint32_t*)_REG(PARAMS)); i++){
          //read input data
          memRead(*((uint32_t*)_REG(ARRAY_IN))
                  + i*(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs))),
                  training_data, sizeof(TYPE_BP)*(training_sets * input_dimension), 0);

          memRead(*((uint32_t*)_REG(ARRAY_IN)) + sizeof(TYPE_BP)*(training_sets * input_dimension)
                  + i*(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs))),
                  training_targets, sizeof(TYPE_BP)*(training_sets * possible_outputs), 0);

          memRead(*((uint32_t*)_REG(ARRAY_IN))
                  + *((uint32_t*)_REG(PARAMS)) *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                  + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                     + possible_outputs)), weights1, sizeof(TYPE_BP)*(nodes_per_layer*input_dimension), 0);

          memRead(*((uint32_t*)_REG(ARRAY_IN)) + sizeof(TYPE_BP)*(nodes_per_layer*input_dimension)
                  + *((uint32_t*)_REG(PARAMS)) *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                  + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                     + possible_outputs)), weights2, sizeof(TYPE_BP)*(nodes_per_layer*nodes_per_layer), 0);

          memRead(*((uint32_t*)_REG(ARRAY_IN)) + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer))
                  + *((uint32_t*)_REG(PARAMS)) *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                  + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                     + possible_outputs)), weights3, sizeof(TYPE_BP)*(nodes_per_layer*possible_outputs), 0);

          memRead(*((uint32_t*)_REG(ARRAY_IN)) + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer + possible_outputs))
                  + *((uint32_t*)_REG(PARAMS)) *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                  + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                     + possible_outputs)), biases1, sizeof(TYPE_BP)*nodes_per_layer, 0);

          memRead(*((uint32_t*)_REG(ARRAY_IN)) + sizeof(TYPE_BP)*(nodes_per_layer*(1+input_dimension+nodes_per_layer + possible_outputs))
                  + *((uint32_t*)_REG(PARAMS)) *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                  + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                     + possible_outputs)), biases2, sizeof(TYPE_BP)*nodes_per_layer, 0);

          memRead(*((uint32_t*)_REG(ARRAY_IN)) + sizeof(TYPE_BP)*(nodes_per_layer*(2+input_dimension+nodes_per_layer + possible_outputs))
                  + *((uint32_t*)_REG(PARAMS)) *(sizeof(TYPE_BP)*(training_sets * (input_dimension + possible_outputs)))
                  + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                     + possible_outputs)), biases3, sizeof(TYPE_BP)*possible_outputs, 0);

          backprop(weights1, weights2, weights3, biases1, biases2, biases3,
                   training_data, training_targets);
          wait(cTime_ns, SC_NS);
          // FIXME: add Computation Energy monitoring feature
          // FIXME: append energy and time cost to monSystem
          //write output
          memWrite(*((uint32_t*)_REG(ARRAY_OUT))
                   + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                      + possible_outputs)), weights1, sizeof(TYPE_BP)*(nodes_per_layer*input_dimension), 0);

          memWrite(*((uint32_t*)_REG(ARRAY_OUT)) + sizeof(TYPE_BP)*(nodes_per_layer*input_dimension)
                  + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                     + possible_outputs)), weights2, sizeof(TYPE_BP)*(nodes_per_layer*nodes_per_layer), 0);

          memWrite(*((uint32_t*)_REG(ARRAY_OUT)) + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer))
                  + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                     + possible_outputs)), weights3, sizeof(TYPE_BP)*(nodes_per_layer*possible_outputs), 0);

          memWrite(*((uint32_t*)_REG(ARRAY_OUT)) + sizeof(TYPE_BP)*(nodes_per_layer*(input_dimension+nodes_per_layer + possible_outputs))
                  + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                     + possible_outputs)), biases1, sizeof(TYPE_BP)*nodes_per_layer, 0);

          memWrite(*((uint32_t*)_REG(ARRAY_OUT)) + sizeof(TYPE_BP)*(nodes_per_layer*(1+input_dimension+nodes_per_layer + possible_outputs))
                  + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                     + possible_outputs)), biases2, sizeof(TYPE_BP)*nodes_per_layer, 0);

          memWrite(*((uint32_t*)_REG(ARRAY_OUT)) + sizeof(TYPE_BP)*(nodes_per_layer*(2+input_dimension+nodes_per_layer + possible_outputs))
                  + i*(sizeof(TYPE_BP)*(nodes_per_layer*(2 + nodes_per_layer + input_dimension +possible_outputs)
                                     + possible_outputs)), biases3, sizeof(TYPE_BP)*possible_outputs, 0);
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
