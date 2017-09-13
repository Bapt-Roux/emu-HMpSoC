#include "scTime.h"

scTime::scTime(sc_module_name name)
  : sc_module(name), time_Ssk("time_Ssk")
{
  time_Ssk.register_b_transport(this, &scTime::b_transport);
}

scTime::~scTime(){
}

void scTime::b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
{
  tlm::tlm_command cmd = trans.get_command();
  sc_dt::uint64    addr = trans.get_address();
  unsigned char*   ptr = trans.get_data_ptr();
  unsigned int     len = trans.get_data_length();
  unsigned char*   byt = trans.get_byte_enable_ptr();

  if (byt != 0) {
    trans.set_response_status(tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE);
    SC_REPORT_FATAL("scTime", "Unsupported access \n");
    return;
  }

  if (len > (sizeof(uint32_t))) {
    trans.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
    SC_REPORT_FATAL("scTime", "Unsupported burst access \n");
    return;
  }

  if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    static uint64_t timeNS = 0;
    static uint64_t clockCnt = 0;
    uint32_t rBuffer=0;

    if (SCTIME_GTIME_NS == addr){ //updt time and send LSB
      timeNS = sc_time_stamp().to_seconds() * SEC_TO_NS;
      rBuffer = (timeNS &0xffffffff);
    } else if((SCTIME_GTIME_NS + sizeof(uint32_t)) == addr){ //send timeMSB
      rBuffer = ((timeNS >>32) &0xffffffff);
    }else if (SCTIME_GCLOCK == addr){ //updt clock counter and send LSB
      clockCnt = clock();
      rBuffer = (clockCnt &0xffffffff);
    } else if((SCTIME_GCLOCK + sizeof(uint32_t)) == addr){ //send clock MSB
      rBuffer = ((clockCnt >>32) &0xffffffff);
    }
    memcpy(ptr, &rBuffer, len);
  } else if (cmd == tlm::TLM_WRITE_COMMAND) {
    if (SCTIME_GTIME_NS == addr){
      if (0x00 == ptr[0]){
        std::cout << name() << ": Stop systemC simulation @"<< sc_time_stamp() << "\n";
        sc_stop();
        exit(1);
      }
    }
  }
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}
