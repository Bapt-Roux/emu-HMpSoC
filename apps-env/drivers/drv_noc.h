/* ----------------------------------------------------------------------------
 * Linux driver for NoC emulation through IPC
 * Author: baptiste.roux AT inria.fr
 *
 * This driver manage NIPC IRQ and process waiting list
 *
 * This driver use device tree facilities to probe available devices and then
 * implement basic file operation as char device driver
 * -------------------------------------------------------------------------- */
#include "noc_hw.h"
#include <linux/wait.h>

/* ---
 * PREPROCESSOR MACCROS
 * ------------------------------------------------------ */
#ifdef NIPC_DEBUG
#define printk_HDBG(format, arg...) pr_info( "[PID 0x%x]=> " DRIVER_NAME ": %s:"  format, current->pid, __FUNCTION__, ## arg)
#else
#define printk_HDBG(format, arg...)
#endif

#define printk_dbg(format, arg...) { if (debug) pr_info( "[PID 0x%x]=> " DRIVER_NAME ": %s:"  format, current->pid, __FUNCTION__, ## arg); }
#define printk_err(format, arg...)   pr_err("[PID 0x%x]=> " DRIVER_NAME ":"  format, current->pid, ## arg)
#define printk_info(format, arg...) pr_info("[PID 0x%x]=> " DRIVER_NAME ":"  format, current->pid, ## arg)
#define printk_warn(format, arg...) pr_warn("[PID 0x%x]=> " DRIVER_NAME ":"  format, current->pid, ## arg)

/* ---
 * DRIVER STRUCT
 * ------------------------------------------------------ */
#define MAX_PRC 6
#define PRC_WAKEUP 0xff
#define PRC_WAIT 0xaa
#define DFLT_FIFO_SIZE 15

typedef struct wStatus{
  uint8_t status;
  struct waitPoint *wp;
}wStatus;

typedef struct orderRcv{
  task_t task_id;
  task_t from_id;
  order_t order_type;
  struct orderHeader *po;
  struct orderRcv *nxtOrder;
}oRcv;

typedef struct prcRegister{
  pid_t prc_id;
  uint8_t nbTask;
  wait_queue_head_t prcQueue;
  oRcv *orderPool;
  wStatus waitState;
  struct prcRegister* nxtPrc;
}pReg;

typedef struct taskRegister{
  task_t task_id;
  pReg* trgtPrc;
  struct taskRegister* nxtTask;
}tReg;
