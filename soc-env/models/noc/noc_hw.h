/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: noc_hw.h
 * NoC register offset
 */

#ifndef _NOC_HW
#define _NOC_HW
/*
 * Address offset definition
 */
#define DEFAULT_NOC_ADDR 0x5ffe0000UL

/*---
 * COMMAND/STATUS/REG_OFFSET BASE DEFINITION
 *----------------------------------------------------------------------------*/
//REGISTER OFFSET
#define RNOC_CMD      0x0000
#define RNOC_RCVORDER 0x0004
#define RNOC_WAKEUP   0x0008
#define RNOC_RCVDATA  0x0010
#define RNOC_SENDDATA 0x0040
#define RNOC_POOLSIZE 0x006F
//CMD VALUE
#define CNOC_SORDER 0x01
#define CNOC_SYNC 0x02
#define CNOC_GORDER 0x10
#define CNOC_WAKEUP 0x20
#define CNOC_DISCOVER 0x40
#define CNOC_NOCPOWER 0x80



/*---
 * DATA EXCHANGE STRUCT DEFINITION
 *----------------------------------------------------------------------------*/
/*
 * base type for NoC packets header
 */
//NocItf endpoint task_id composition
#define NOC_TASKMASK 0xfffff000
#define NOC_JOBMASK  0xff000000
#define NOC_PCKTMASK 0x00000fff
#define MGMT_TASKID  NOC_TASKMASK
//task_id: [   NOC_TASKMASK   , NOC_PCKTMASK ]
// meaning [[job_id, worker_id], packet_id(LSB)]
typedef uint32_t task_t;

#define NOC_DEVICE "/dev/sc_noc"
#define TASK_TO_TID 24
#define TASK_IDOFFSET 12
#define HWFLAG 0x40000000


typedef uint8_t ack_t;
#define UNSET_ACK 0x00
#define NACK 0x01
#define RCV_ACK 0x04
#define PRC_ACK 0x10

typedef enum {NOC_FAILED=0x00, NOC_JOBASK=0x01, NOC_MEMGET=0x02, NOC_MEMRTV=0x04, NOC_JOBSYNC=0x08,
      NOC_ACK=0x10, NOC_POWERLOG=0xFF}order_t;

/*
 * Struct for waiting point definition
 * task_id=> target task event
 * ack_type => target ack type that generate a wakeup
 */
struct waitPoint{
  task_t task_id;
  ack_t ack_type;
};
/*
 * Struct for help wait on Order definition:
 *                      ease communication between userSpace API and driver
 * job_id=> target job
 * order_type => target order
 */
struct waitOrder{
  task_t task_id;
  task_t from_id;
  order_t order_type;
  struct orderHeader *data;
};
/*
 * Definition for waitpoint request status
 */
typedef uint8_t waitStatus;
#define WAITPOINT_SET 0xC4

/*
 * Struct for sending or received order
 */
struct orderHeader{
  // bool is_sync_header=false;
  bool is_sync_header;
  task_t task_id;
  task_t sender_id;
  // order content
  order_t cmd;
  uint64_t local; // only relevant with rtv_cmd
  uint64_t remote;
  uint32_t data_length;
  // double deltaInCycle_ns=0.0; // TLM time sync
  double deltaInCycle_ns; // TLM time sync
};

/*
 * Struct for sending or received callback data from order
 */
struct dcbHeader{
  // bool is_sync_header=false;
  bool is_sync_header;
  // callback content
  uint64_t remote;
  uint32_t data_length;
  // double deltaInCycle_ns=0.0; //TLM time sync
  double deltaInCycle_ns; //TLM time sync
};


/***
 * NoC Topology
 * Note: position display as [y,x]
 */

typedef struct {
  uint8_t sk;
  uint8_t x_pos;
  uint8_t y_pos;
}clPos;
#endif /*_NOC_HW*/
