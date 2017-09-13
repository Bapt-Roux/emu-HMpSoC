/**
 * @file md_knn_hws.cc
 * @brief HW Stub for MD_KNN from machsuite.
 */
#include "md_knn_hws.h"

/**
 * @brief emu-hmpsoc machsuite md_knn HW_Stub constructor.
 * @params name: systemC entity name.
 * @params compTime_ns: execution Time[ns] of one iteration.
 * @params compTime_ns: execution Energy[nJ] of one iteration.
 */
md_knn_hwStub::md_knn_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj)
  : hwIP(name, (uint8_t)1 /*One HPx*/, (uint8_t)1 /*One irq*/, GENIP_RPSIZE),
    cTime_ns(compTime_ns), cEn_nj(compEn_nj)
{
  //Register THREAD
  SC_THREAD(parse_cmd);
  sensitive << hwIP::cmd_events;
}

md_knn_hwStub::~md_knn_hwStub(){
}

/**
 * @brief emu-hmpsoc SC_Thread implementation for machsuite md_knn HW_Stub .
 */
void md_knn_hwStub::parse_cmd(){
  wait(); // prevent execution at bootup
  while(1) {
#ifdef MACHSUITE_DEBUG
    std::cout << "@ "<< sc_time_stamp() << " ==>"<< "MD_KNN received cmd." << std::endl;
#endif
    switch (*((uint8_t*)_REG(CMD)))
      {
      case CMD_START:
        irqn[0].write(0); // reset irq Done
        *((uint8_t*)_REG(STATUS)) &= ~(STATUS_DONE + STATUS_IRQ(0));
#ifdef MACHSUITE_DEBUG
        std::cout << "MD_KNN_HW started for "<< (uint32_t) *((uint32_t*)_REG(PARAMS)) <<" iterations\n";
        std::cout << "ARRAY IN 0x"<<std::hex << (uintptr_t) *((uint32_t*)_REG(ARRAY_IN)) <<"\n";
        std::cout << "ARRAY OUT 0x"<<std::hex << (uintptr_t) *((uint32_t*)_REG(ARRAY_OUT)) <<"\n";
#endif
        for( uint i=0; i< *((uint32_t*)_REG(PARAMS)); i++){
          //read input data
          memRead(*((uint32_t*)_REG(ARRAY_IN))
                  + i*(sizeof(TYPE_MDK)* (3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors)), xp, sizeof(TYPE_MDK)*(nAtoms), 0);
          memRead(*((uint32_t*)_REG(ARRAY_IN)) + sizeof(TYPE_MDK)* (nAtoms)
                  + i*(sizeof(TYPE_MDK)* (3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors)), yp, sizeof(TYPE_MDK)*(nAtoms), 0);
          memRead(*((uint32_t*)_REG(ARRAY_IN)) + sizeof(TYPE_MDK)* (2*nAtoms)
                  + i*(sizeof(TYPE_MDK)* (3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors)), zp, sizeof(TYPE_MDK)*(nAtoms), 0);
          memRead(*((uint32_t*)_REG(ARRAY_IN)) + sizeof(TYPE_MDK)* (3*nAtoms)
                  + i*(sizeof(TYPE_MDK)* (3*nAtoms) + sizeof(int32_t)*(nAtoms*maxNeighbors)),
                  neighbors, sizeof(int32_t)*(nAtoms*maxNeighbors), 0);
          md_kernel(xf, yf ,zf,
                    xp, yp, zp, neighbors);
          wait(cTime_ns, SC_NS);
          // FIXME: add Computation Energy monitoring feature
          // FIXME: append energy and time cost to monSystem
          //write output
          memWrite(*((uint32_t*)_REG(ARRAY_OUT))
                   + i*(sizeof(TYPE_MDK)* (3*nAtoms)), xf, sizeof(TYPE_MDK)*(nAtoms), 0);
          memWrite(*((uint32_t*)_REG(ARRAY_OUT)) + sizeof(TYPE_MDK)* (nAtoms)
                   + i*(sizeof(TYPE_MDK)* (3*nAtoms)), yf, sizeof(TYPE_MDK)*(nAtoms), 0);
          memWrite(*((uint32_t*)_REG(ARRAY_OUT)) + sizeof(TYPE_MDK)* (2*nAtoms)
                   + i*(sizeof(TYPE_MDK)* (3*nAtoms)), zf, sizeof(TYPE_MDK)*(nAtoms), 0);
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
