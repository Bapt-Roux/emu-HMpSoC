/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: nocIpc.h
 * Noc for inter systemC kernel communication
 */

#ifndef _NOC_IPC
#define _NOC_IPC
#include "nocBase.h"
#include "poll.h"

#define NS_TO_S (float)(0.000000001)
#define NOC_POLL_TIMEOUT_MS -1
//Debug FLAGS management
#ifdef NIPC_DEBUG
// #define NIPC_DEBUG_IPC  //IPC layer
// #define NIPC_DEBUG_SYNC //Sync algorithm
// #define NIPC_DEBUG_BIND //Cluster Binding mechanism
// #define NIPC_DEBUG_TRANS //Transport layer
#define NIPC_DEBUG_CPU  //Storing and Ack request mechanisms
#endif

#define MAX_CB_PAYLOAD_LEN 0x100000

namespace noc{
  namespace ipc{
    typedef struct sockaddr_un sockaddrUnix;

    uint erFree_recv(int socket, char* trgt, size_t size);
    uint erFree_send(int socket, char* src, size_t size);

    /*
     * ipc implementation of nocItf class
     * Class with TLM socket and IPC socket
     * Do traduction between the 2 world for multiThread simulation
     */
    class nocItf: public noc::nocItf_base
    {
      void cpuCmdEvent();
      void b_transport(tlm::tlm_generic_payload& trans, sc_time& delay);

      //noc synchronization step
      uint preBoot_ms;
      uint postBoot_us;

      sc_event syncEvt;
      int sockfd;
      sockaddrUnix *trgtAddr;

      //data callback phase
      char *cb_payload;

      //Apply deltaInCycle on IRQ generation
      sc_semaphore irqOrder_pending;
      sc_event orderEvt_rcv;
      sc_semaphore irqWake_pending;
      sc_event wakeEvt_rcv;
      void order_irqRcvEvent();
      void wake_irqRcvEvent();
      //Instead of order waitPoint doesn't have this field,
      //stock it in another vector to keep the interface unchanged and compatible with noc_mono implementation
      std::vector<double> wakeupDeltaInCycle;

    public:
      SC_HAS_PROCESS(nocItf);
      nocItf(sc_core::sc_module_name name, std::map<uint, noc::clPos>nocDiscover, uint preboot_ms, uint postboot_us);
      ~nocItf();
      void openUnixIpc(std::string socketPath);
      void closeUnixIpc();
      void syncUnixMsg();
    };

    class nocRouter: public noc::nocRouter_base
    {
      int mainfd;
      sockaddrUnix *mainAddr;
      sockaddrUnix *sockAddr;
      int *sockfd;
      struct pollfd *fdRs;
      bool routerEnable;

      //data callback payload
      char *cb_payload;

    public:
      nocRouter(int nbSk, std::string socketPath, sb::nocCnf config);
      ~nocRouter();
      void startRouter();
      uint bindToCluster(tlm_utils::simple_target_socket<nocItf_base>* nocItf_Ssk,
                         tlm_utils::simple_initiator_socket<nocItf_base>* nocItf_Msk,
                         uint64_t clBaseAddr, uint8_t cl_x, uint8_t cl_y);
    };

  }// end namespace ipc
}// end namespace noc
#endif /*_NOC_IPC*/
