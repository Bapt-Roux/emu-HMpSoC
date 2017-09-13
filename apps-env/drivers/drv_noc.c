/* ----------------------------------------------------------------------------
 * Linux driver for NoC emulation through IPC
 * Author: baptiste.roux AT inria.fr
 *
 * This driver manage NOC IRQ and process waiting list
 *
 * This driver use device tree facilities to probe available devices and then
 * implement basic file operation as char device driver
 * -------------------------------------------------------------------------- */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>

#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/cdev.h>

#include <linux/interrupt.h>
#include <linux/of_irq.h>

#include <linux/sched.h>

#include <linux/kfifo.h>

#include "drv_noc.h"
#include "cmd_noc.h"

#define  DEVICE_NAME "noc"
#define  DRIVER_NAME "drv_noc"
#define  CLASS_NAME  "noc"
#define MAX_DEVICES 10

//helper for accessing nocItf register
#define _RNOC(NAME) (((void *)drvdata->noc_ptr+ (RNOC_##NAME)))

MODULE_LICENSE("GPL");
MODULE_AUTHOR("baptiste.roux AT inria.fr");
MODULE_DESCRIPTION("NoC manager for virtual platform NoC IPC emulation");
MODULE_VERSION("1.0");

/* ---
 * MODULE PARAMETERS
 * ------------------------------------------------------ */
static bool debug = false; /* print extra debug info */
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "enable debug info (default: false)");

/* ---
 * HELP MESSAGE DISPLAY ON READ
 * -------------------------------------------------------------------------- */
const char help_read[]="Use ioctl cmd to use this NoC manager\n\
                          NOC_DISCOVER \n\
                          NOC_REGISTER  \n\
                          NOC_UNREGISTER\n\
                          NOC_SEND      \n\
                          NOC_WAITORDER \n\
                          NOC_WAITACK   \n\
                          NOC_GETPOWER  \n\0";
static size_t data_read;


/* ---
 * GLOBAL DEFINE
 * ------------------------------------------------------ */
static struct class *nocIpc_class;
static dev_t device_devt;
static int cur_devno=0;
static const struct of_device_id nocIpc_of_match[] = {
  { .compatible = "emu-hmpsoc,nocIpc-1.0", },
  {}
};
MODULE_DEVICE_TABLE(of, nocIpc_of_match);

struct nocIpc_drvdata {
  u8 * dev_name;//UTILITY
  u32 major; u32 minor;
  struct cdev cdev;//CHRDEV REGISTER struct
  struct device *dev;
  u32 cursor;

  struct mutex nocIpc_mutex;
  phys_addr_t noc_reg;
  resource_size_t noc_size;
  void __iomem *noc_ptr;

  // internal data struct:
  // list of task registered and list of attach process
  tReg *regTasks;
  pReg *regPrcs;

  //topHalf to bottomHalf IRQ communication
  u32 irq_num[2]; //irq[0]: wakeUp; irq[1]: order;
  //NOTE: kfifo_alloc failed on struct orderHeader: use generic u32
  // type with the right number of slot instead
  DECLARE_KFIFO_PTR(orderFifo, u32);
  DECLARE_KFIFO_PTR(ackFifo, struct waitPoint);
  struct tasklet_struct order_tasklet;
  struct tasklet_struct ack_tasklet;
  };
#define SORDERH_u32 (sizeof(struct orderHeader)/sizeof(u32))

/* ---
 * Driver function prototypes and registration in kernel
 * file structure
 * ----------------------------------------------------- */
static int nocIpc_open (struct inode *inode, struct file *file);
static int nocIpc_release (struct inode *inode, struct file *file);
static ssize_t nocIpc_read (struct file *file, char __user *user_buf, size_t size, loff_t *offset);
static ssize_t nocIpc_write (struct file *, const char __user *user_buf, size_t size, loff_t *offset);
static long nocIpc_ioctl(struct file *, unsigned int, unsigned long);
static irqreturn_t noc_isr_order(int irq, void *dev_id, struct pt_regs *regs);
static irqreturn_t noc_isr_wakeUp(int irq, void *dev_id, struct pt_regs *regs);
static void _order_tasklet(struct nocIpc_drvdata *drvdata);
static void _ack_tasklet(struct nocIpc_drvdata *drvdata);

static struct file_operations fops = {
  .open = nocIpc_open,
  .release = nocIpc_release,
  .read = nocIpc_read,
  .write = nocIpc_write,
  .compat_ioctl = nocIpc_ioctl,
  .unlocked_ioctl = nocIpc_ioctl
};


/* ---
 * Probing function prototypes and registration for
 * platform devices
 * ----------------------------------------------------- */
static int nocIpc_probe(struct platform_device *pdev);
static int nocIpc_remove(struct platform_device *pdev);

static struct platform_driver platform_driver = {
  .probe = nocIpc_probe,
  .remove = nocIpc_remove,
  .driver = {
    .name = DRIVER_NAME,
    .of_match_table = nocIpc_of_match,
  },
};

static int nocIpc_probe(struct platform_device *pdev)
{
  struct nocIpc_drvdata * drvdata;
  struct resource res;
  s32 retval =0;
  dev_t devt;
  /*specific dt property index*/
  const u8 *spec_prop_ptr;
  int spec_prop_len =0;
  int i ;

  printk_dbg("Call probing function\n");

  /*sanity check */
  const struct of_device_id *match;
  match = of_match_device(nocIpc_of_match, &pdev->dev);
  if (!match)
    {
      printk_err("Device doesn't match driver compatible\n");
      retval = -EINVAL;
      goto exit_0;
    }

  /*allocate drvdata memory */
  drvdata = kzalloc(sizeof(struct nocIpc_drvdata), GFP_KERNEL);
  if (!drvdata) {
    printk_err("Couldn't allocate device private record\n");
    retval = -ENOMEM;
    goto exit_0;
  }
  dev_set_drvdata(&pdev->dev, (void *)drvdata);

  /*set devno and pointer to device*/
  drvdata->major = MAJOR(device_devt);
  drvdata->minor = cur_devno++;
  devt = MKDEV(drvdata->major, drvdata->minor);
  drvdata->dev = &pdev->dev;

  /*read device information */
  retval = of_address_to_resource(pdev->dev.of_node, 0, &res);
  if(retval){
    printk_err("Can't access register values \n");
    goto exit_0;
  }
  drvdata->noc_reg = res.start;
  drvdata->noc_size =resource_size(&res);

  /* read dtb irq info */
  drvdata->irq_num[0] = irq_of_parse_and_map(pdev->dev.of_node, 0);
  drvdata->irq_num[1] = irq_of_parse_and_map(pdev->dev.of_node, 1);
  if( !(drvdata->irq_num[0]) || !(drvdata->irq_num[1])){
    printk_err("Can't access irq values: first[wakeUp_line]; second[order_line]\n");
    retval = -1;
    goto exit_0;
  }

  u32 fifo_size;
  if (of_property_read_u32(pdev->dev.of_node, "fifo-size", &fifo_size)){
    printk_warn("Couldn't determine fifo-size information, set to default values %d",
                DFLT_FIFO_SIZE);
    fifo_size = DFLT_FIFO_SIZE;
  }

  retval = kfifo_alloc(&(drvdata->orderFifo), SORDERH_u32*fifo_size, GFP_KERNEL);
  if (retval) {
    printk_err("error kfifo_alloc for orderFifo\n");
    return retval;
  }
  retval = kfifo_alloc(&(drvdata->ackFifo), fifo_size, GFP_KERNEL);
  if (retval) {
    printk_err("error kfifo_alloc for ackFifo\n");
    return retval;
  }

  /* read dev-name info */
  spec_prop_ptr = of_get_property(pdev->dev.of_node, "dev-name",&spec_prop_len);
  if( NULL == spec_prop_ptr){
    printk_warn("Couldn't determine dev_name information, set to default values");
    drvdata->dev_name = vmalloc(6*sizeof(u8));
    drvdata->dev_name[0]='s'; drvdata->dev_name[1]='c'; drvdata->dev_name[2]='n';
    drvdata->dev_name[3]='o'; drvdata->dev_name[4]='c';drvdata->dev_name[5]='\0';
  }else{
  drvdata->dev_name = vmalloc(spec_prop_len*sizeof(u8));
  for(i=0 ; i<spec_prop_len ; i++)
    (*(drvdata->dev_name +i)) = (char)(*(spec_prop_ptr+i));
  (*(drvdata->dev_name +spec_prop_len)) = (char)'\0';
  }

  /* register device as char device */
  cdev_init(&drvdata->cdev, &fops);
  drvdata->cdev.owner = THIS_MODULE;
  retval = cdev_add(&drvdata->cdev, devt, 1);
  if (retval) {
    dev_err(&pdev->dev, "cdev_add() failed\n");
    goto exit_1;
  }

  /*Request and map memory */
  if (!request_mem_region(drvdata->noc_reg, drvdata->noc_size, drvdata->dev_name)) {
    printk_err("Can't request memory\n");
    retval = -ENOMEM;
    goto exit_1;
  }
  drvdata->noc_ptr = of_iomap(pdev->dev.of_node, 0);
  if(!(drvdata->noc_ptr)){
    printk_err("Can't do iomap\n");
    retval = -EFAULT;
    goto exit_2;
  }

  /*Ask for irq */
  retval = request_irq(drvdata->irq_num[0], (irq_handler_t) noc_isr_wakeUp,
                       IRQF_NOBALANCING, drvdata->dev_name, drvdata);
  if(retval){
    printk_err("Can't request irq for ack_line\n");
    goto exit_3;
  }
  retval = request_irq(drvdata->irq_num[1], (irq_handler_t) noc_isr_order,
                       IRQF_NOBALANCING, drvdata->dev_name, drvdata);
  if(retval){
    printk_err("Can't request irq for order_line\n");
    goto exit_4;
  }

  /*Initialize tasklet with data pointing on drvdata */
  tasklet_init(&drvdata->order_tasklet,
               (void(*)(unsigned long))_order_tasklet,
               (unsigned long)drvdata);
  tasklet_init(&drvdata->ack_tasklet,
               (void(*)(unsigned long))_ack_tasklet,
               (unsigned long)drvdata);

  /* Initialize device vars */
  mutex_init(&drvdata->nocIpc_mutex);
  drvdata->regTasks = NULL;
  drvdata->regPrcs = NULL;

  /*dump value*/
  printk_dbg("dev-name: %s \n", drvdata->dev_name);
  printk_dbg("register values read are: 0x%x  0x%x \n",
             drvdata->noc_reg, drvdata->noc_size);

  device_create(nocIpc_class, &pdev->dev, devt, NULL, "%s",drvdata->dev_name);
  return retval;

 exit_4:
  free_irq(drvdata->irq_num[1], &pdev->dev);
 exit_3:
  free_irq(drvdata->irq_num[0], &pdev->dev);
 exit_2:
  iounmap(drvdata->noc_ptr);
 exit_1:
  vfree(drvdata->dev_name);
 exit_0:
  kfree(drvdata);
  return retval;
}

static int nocIpc_remove(struct platform_device *pdev)
{
  struct nocIpc_drvdata *drvdata;

  printk_dbg("Call remove function\n");

  drvdata = dev_get_drvdata(&pdev->dev);
  if(!drvdata)
    return 0;

  /*unregister device */
  mutex_destroy(&drvdata->nocIpc_mutex);
  cdev_del(&drvdata->cdev);
  iounmap(drvdata->noc_ptr);
  release_mem_region(drvdata->noc_reg, drvdata->noc_size);
  free_irq(drvdata->irq_num[1], drvdata);
  free_irq(drvdata->irq_num[0], drvdata);
  kfifo_free(&(drvdata->ackFifo));
  kfifo_free(&(drvdata->orderFifo));
  tasklet_kill(&(drvdata->ack_tasklet));
  tasklet_kill(&(drvdata->order_tasklet));
  vfree(drvdata->dev_name);
  kfree(drvdata);
  return 0;
}

/* ---
 * Init and Exit driver function
 * ------------------------------------------------------ */
static int __init nocIpc_init(void)
{
  int retval = 0;

  printk_info("Register platform device \n");

  nocIpc_class = class_create(THIS_MODULE, "nocIpc_drv");
  nocIpc_class->dev_groups = NULL;
  retval = alloc_chrdev_region(&device_devt, 0, MAX_DEVICES,DRIVER_NAME);
  if (retval < 0)
    {
      printk_err("Couldn't allocate MAJOR number \n");
      return retval;
    }
  data_read  = strlen(help_read);
  retval = platform_driver_register(&platform_driver);

  return retval;
}

static void __exit nocIpc_exit(void)
{
  int i=0;
  dev_t devt;

  printk_info("UnRegister platform device \n");

  for(i=0 ; i<cur_devno ; i++){
    devt = MKDEV(MAJOR(device_devt), MINOR(device_devt)+i);
    device_destroy(nocIpc_class, devt);
  }
  class_destroy(nocIpc_class);
  platform_driver_unregister(&platform_driver);
  unregister_chrdev_region(device_devt, MAX_DEVICES);


}
module_init(nocIpc_init);
module_exit(nocIpc_exit);


/* ---
 * Driver function implementation
 * ----------------------------------------------------- */
static int nocIpc_open (struct inode *inode, struct file *file)
{
  struct nocIpc_drvdata *drvdata;
  drvdata = container_of(inode->i_cdev, struct nocIpc_drvdata, cdev);
  file->private_data = drvdata;
  drvdata->cursor = 0;//set read/Write cursor to 0

  printk_dbg("OPEN DEVICE \n");
  try_module_get(THIS_MODULE);
  return 0;
}

static int nocIpc_release (struct inode *inode, struct file *file)
{
  struct nocIpc_drvdata *drvdata = file->private_data;

  printk_dbg("RELEASE DEVICE \n");
  module_put(THIS_MODULE);
  return 0;
}

static ssize_t nocIpc_read (struct file *file, char __user *user_buf, size_t size, loff_t *offset)
{
  struct nocIpc_drvdata *drvdata = file->private_data;
  size_t n;

  mutex_lock_interruptible(&drvdata->nocIpc_mutex);
  if(drvdata->cursor >= data_read) { //all data has been read i.e EOF
    if(copy_to_user(user_buf, "\0", 1)){return -EFAULT;}
    return 0;
  }
  n = data_read - drvdata->cursor; //remainder
  n = (size >= n ? n : size);
  if(copy_to_user(user_buf, help_read+(drvdata->cursor), n)){return -EFAULT;}
  if(copy_to_user(user_buf+n, "\0", 1)){return -EFAULT;}
  drvdata->cursor += n;
  mutex_unlock(&drvdata->nocIpc_mutex);
  return n; //bytes_read;
}

static ssize_t nocIpc_write (struct file *file, const char __user *user_buf, size_t size, loff_t *offset)
{
  printk_info("No writing method implemented, use ioctl instead.\n");
  return -1;
}


/* ---
 * Utility functions to search in driver linked chain
 --- */
// return 1 if task exist with a cur pointing to it, else
//    return 0 with pointer on last elements
// prv always point to previous element
u8 findTask(task_t task_id, tReg *origin, tReg **prv, tReg **cur){
  (*prv) = NULL;
  (*cur) = origin;

  if(NULL == origin){//empty list
    return(0);
  }else{
    while(NULL != (*cur)->nxtTask){
      if(task_id == (*cur)->task_id){//found
        return(1);
      }else{
        *prv = *cur;
        (*cur) = (*cur)->nxtTask;
      }
    }
    // Check last element of list
    if(task_id == (*cur)->task_id){//found
      return(1);
    }else{
      return(0);
    }
  }
}
// return 1 if prc exist with a cur pointing to it, else
//    return 0 with pointer on last process
// prv always point to previous element
u8 findPrc(pid_t prc_id, pReg *origin, pReg **prv, pReg **cur){
  (*prv) = NULL;
  (*cur) = origin;

  if(NULL == origin){//empty list
    return(0);
  }else{
    while(NULL != (*cur)->nxtPrc){
      if(prc_id == (*cur)->prc_id){//found
        return(1);
      }else{
        *prv = *cur;
        (*cur) = (*cur)->nxtPrc;
      }
    }
    // Check last element of list
    if(prc_id == (*cur)->prc_id){//found
      return(1);
    }else{
      return(0);
    }
  }
}

// return 1 if order exist with a cur pointing to it, else
//    return 0 with pointer on last order
// prv always point to previous element
u8 findOrder(task_t task_id, task_t from_id, order_t order_type, oRcv *origin, oRcv **prv, oRcv **cur){
  (*prv) = NULL;
  (*cur) = origin;

  if(NULL == origin){//empty list
    return(0);
  }else{
    while(NULL != (*cur)->nxtOrder){
      if((task_id == (*cur)->task_id) && (from_id == (*cur)->from_id) && (order_type == (*cur)->order_type) ){//found
        return(1);
      }else{
        *prv = *cur;
        (*cur) = (*cur)->nxtOrder;
      }
    }
    // Check last element of list
    if((task_id == (*cur)->task_id) && (from_id == (*cur)->from_id) && (order_type == (*cur)->order_type) ){//found
      return(1);
    }else{
      return(0);
    }
  }
}


static long nocIpc_ioctl(struct file *file, unsigned int nocIpc_cmd, unsigned long ioctl_arg)
{
  struct nocIpc_drvdata *drvdata = file->private_data;

  //user communication buffer
  u32 *userBuffer;
  u32 userTask;
  u16 i;
  s16 status=0;
  tReg* curTask, *prvTask;
  pReg* curPrc, *prvPrc;
  oRcv* curOrd, *prvOrd;

  mutex_lock_interruptible(&drvdata->nocIpc_mutex);

  curTask = drvdata->regTasks;
  printk_HDBG("Register Task status:---------------------------------------\n");
  if(NULL == curTask){
    printk_HDBG("\t EMPTY.\n");
  }else{
    while(NULL != curTask->nxtTask){
      printk_HDBG("\t task 0x%x registered on prc 0x%x;\n",
                  curTask->task_id, curTask->trgtPrc->prc_id);
      curTask = curTask->nxtTask;
    }
    printk_HDBG("\t task 0x%x registered on prc 0x%x.\n",
                curTask->task_id, curTask->trgtPrc->prc_id);
  }
  printk_HDBG("PROCESS status:---------------------------------------------\n");
  curPrc = drvdata->regPrcs;
  if(NULL == curPrc){
    printk_HDBG("\t EMPTY.\n");
  }else{
    while(NULL != curPrc->nxtPrc){
      printk_HDBG("\t prc 0x%x [%d]\n", curPrc->prc_id, curPrc->nbTask);
      curOrd = curPrc->orderPool;
      if(NULL == curOrd){
        printk_HDBG("\t\t order empty.\n");
      }else{
        while(NULL != curOrd->nxtOrder){
          if(NULL == curOrd->po)
            printk_HDBG("\t\t orderW 0x%x; cmd 0x%x:\n", curOrd->task_id, curOrd->order_type);
          else
            printk_HDBG("\t\t orderR 0x%x; cmd 0x%x:\n", curOrd->po->task_id, curOrd->order_type);
          curOrd = curOrd->nxtOrder;
        }
        if(NULL == curOrd->po)
          printk_HDBG("\t\t orderW 0x%x; cmd 0x%x:\n", curOrd->task_id, curOrd->order_type);
        else
          printk_HDBG("\t\t orderR 0x%x; cmd 0x%x:\n", curOrd->po->task_id, curOrd->order_type);
      }
      curPrc = curPrc->nxtPrc;
    }
    printk_HDBG("\t prc 0x%x [%d]\n", curPrc->prc_id, curPrc->nbTask);
    curOrd = curPrc->orderPool;
    if(NULL == curOrd){
      printk_HDBG("\t\t order empty.\n");
    }else{
      while(NULL != curOrd->nxtOrder){
        if(NULL == curOrd->po)
          printk_HDBG("\t\t orderW 0x%x; cmd 0x%x:\n", curOrd->task_id, curOrd->order_type);
        else
          printk_HDBG("\t\t orderR 0x%x; cmd 0x%x:\n", curOrd->po->task_id, curOrd->order_type);
        curOrd = curOrd->nxtOrder;
      }
      if(NULL == curOrd->po)
        printk_HDBG("\t\t orderW 0x%x; cmd 0x%x:\n", curOrd->task_id, curOrd->order_type);
      else
        printk_HDBG("\t\t orderR 0x%x; cmd 0x%x:\n", curOrd->po->task_id, curOrd->order_type);
    }
  }

  printk_dbg("cmd=%d\n", nocIpc_cmd);
  switch(nocIpc_cmd){
  case NOC_DISCOVER:{
    u32 nbClusters=0;
    // read numbers of available clusters
    while(0x00 != readb(_RNOC(CMD))); // wait Noc rdy
    writeb(CNOC_DISCOVER, _RNOC(CMD));
    while(0x00 != readb(_RNOC(CMD)));
    nbClusters = readl(_RNOC(SENDDATA));

    //read clusters Addr and position
    userBuffer = vmalloc((1+2*nbClusters)*sizeof(u32));
    userBuffer[0] = nbClusters;
    for( i=1; i<=(2*nbClusters); i++){
      *((u32*)userBuffer+i) = readl(_RNOC(SENDDATA)+(i*sizeof(u32)));
    }
    //send it to user
    if(copy_to_user((void *) ioctl_arg, (void *)userBuffer, (1+2*nbClusters)*sizeof(u32))){return -EFAULT;}
    // release memory
    vfree(userBuffer);
    break;
  };
  case NOC_REGISTER:{
    //read task_id from user
    if(copy_from_user((void *)&userTask, (void *) ioctl_arg, sizeof(u32))){return -EFAULT;}
    userTask &= NOC_TASKMASK;

    // Check if not already registered
    if( findTask(userTask, drvdata->regTasks, &prvTask, &curTask)){
      printk_warn("Current task [0x%x] are already registered\n", userTask);
      status =-1;
    }else{
      if (NULL == curTask){ //list empty
        drvdata->regTasks = vmalloc(sizeof(tReg));
        curTask = drvdata->regTasks;
      }else{ //add element in queue
        curTask->nxtTask = vmalloc(sizeof(tReg));
        curTask = curTask->nxtTask;
      }
      //init task properties
      curTask->task_id = userTask;
      curTask->nxtTask = NULL;
      if(!findPrc(current->pid, drvdata->regPrcs, &prvPrc, &curPrc)){ //create new prc entry
        if (NULL == curPrc){ //list empty
          drvdata->regPrcs = vmalloc(sizeof(pReg));
          curPrc = drvdata->regPrcs;
        }else{ //add element in queue
          curPrc->nxtPrc = vmalloc(sizeof(pReg));
          curPrc = curPrc->nxtPrc;
        }
        //init process properties
        curPrc->prc_id = current->pid;
        curPrc->nbTask = 1;
        init_waitqueue_head(&(curPrc->prcQueue));
        curPrc->orderPool = NULL;
        curPrc->waitState.status = PRC_WAKEUP;
        curPrc->nxtPrc = NULL;
      }else{ //inc nbTask counter
        curPrc->nbTask += curPrc->nbTask;
      }
      // linked task with matching process
      curTask->trgtPrc = curPrc;
    }
    break;
  };
  case NOC_UNREGISTER:{
    //read task_id from user
    if(copy_from_user((void *)&userTask, (void *) ioctl_arg, sizeof(u32))){return -EFAULT;}
    userTask &= NOC_TASKMASK;

    if( !findTask(userTask, drvdata->regTasks, &prvTask, &curTask)){
      printk_warn("Current task [0x%x] not registered \n", userTask);
      status =-1;
    }else{ // remove it
      curPrc = curTask->trgtPrc;
      // remake linked around
      if (NULL == prvTask){ //elmnt in head
        drvdata->regTasks = curTask->nxtTask;
      }else{
        prvTask->nxtTask = curTask->nxtTask;
      }
      vfree(curTask);

      // update process entry
      curPrc->nbTask -=1;
      if (0 == curPrc->nbTask){ //remove process entry
        findPrc(curPrc->prc_id, drvdata->regPrcs, &prvPrc, &curPrc); //search prv prc entry
          if (NULL == prvPrc){ //elmnt in head
            drvdata->regPrcs = curPrc->nxtPrc;
          }else{
            prvPrc->nxtPrc = curPrc->nxtPrc;
          }
          if(NULL != curPrc->orderPool){//clear orderPool
            curOrd = curPrc->orderPool;
            while(NULL != curOrd->nxtOrder){
              prvOrd = curOrd;
              curOrd = curOrd->nxtOrder;
              kfree(prvOrd);
            }
            kfree(curOrd);
          }
        vfree(curPrc);
      }
    }
    break;
  };
  case NOC_SEND:{
    struct orderHeader orderBuffer;
    //read order from user
    if(copy_from_user((void *)&orderBuffer, (void *) ioctl_arg, sizeof(struct orderHeader))){return -EFAULT;}

    while(0x00 != readb(_RNOC(CMD))); // wait Noc rdy
    for( i=0; i< sizeof(struct orderHeader); i+=4){ //write order in nocItf memory
      writel(*((u32*)((void*)&orderBuffer+i)), _RNOC(RCVDATA) + i);
    }
    writeb(CNOC_SORDER, _RNOC(CMD));
    while(0x00 != readb(_RNOC(CMD))); // wait Noc rdy
    printk_HDBG("Order sent to nocItf\n");
    printk_HDBG("is_sync_header 0x%x\n", orderBuffer.is_sync_header);
    printk_HDBG("task_id 0x%x\n", orderBuffer.task_id);
    printk_HDBG("cmd 0x%x\n", orderBuffer.cmd);
    printk_HDBG("local 0x%lx\n", (ulong)orderBuffer.local);
    printk_HDBG("remote 0x%lx\n", (ulong)orderBuffer.remote);
    printk_HDBG("data_length 0x%x\n", orderBuffer.data_length);

    break;
  };
  case NOC_WAITACK:{
    struct waitPoint waitAck;
    waitStatus wStatus;
    //read waitpoint property from user
    if(copy_from_user((void *)&waitAck, (void *) ioctl_arg, sizeof(struct waitPoint))){return -EFAULT;}

    if( !findTask((waitAck.task_id & NOC_TASKMASK), drvdata->regTasks, &prvTask, &curTask)){
      printk_warn("Current task [0x%x] not registered \n", userTask);
      status =-1;
    }else{
      tasklet_disable(&(drvdata->ack_tasklet));
      //register WaitPoint in nocItf
      while(0x00 != readb(_RNOC(CMD))); // wait Noc rdy
      for( i=0; i< sizeof(struct waitPoint); i+=4){ //write waitPoint in nocItf memory
        writel(*((u32*)((void*)&waitAck+i)), _RNOC(RCVDATA) + i);
      }
      writeb(CNOC_SYNC, _RNOC(CMD));
      while(0x00 != readb(_RNOC(CMD))); // wait Noc rdy
      wStatus = readb(_RNOC(SENDDATA));

      if ( WAITPOINT_SET != wStatus){ //Check return value
          waitAck.ack_type = wStatus;
          if(copy_to_user((void *) ioctl_arg,
                          (void *)&waitAck, sizeof(struct waitPoint))){return -EFAULT;}
        tasklet_enable(&(drvdata->ack_tasklet));
      }else{ //setup prc wait flag
        curPrc = curTask->trgtPrc;
        curPrc->waitState.status = PRC_WAIT;
        tasklet_enable(&(drvdata->ack_tasklet));

        mutex_unlock(&drvdata->nocIpc_mutex);
        wait_event_interruptible(curPrc->prcQueue, (PRC_WAKEUP == curPrc->waitState.status));
        //return of sleep
        mutex_lock_interruptible(&drvdata->nocIpc_mutex);
        if(copy_to_user((void *) ioctl_arg,
                          (void *)curPrc->waitState.wp, sizeof(struct waitPoint))){return -EFAULT;}
        kfree(curPrc->waitState.wp);
      }
    }
    break;
  };
  case NOC_WAITORDER:{
    struct waitOrder waitOrder;
    //read waitOrder property from user
    if(copy_from_user((void *)&waitOrder, (void *) ioctl_arg, sizeof(struct waitOrder))){return -EFAULT;}

    //find linked process
    if( !findTask((waitOrder.task_id & NOC_TASKMASK), drvdata->regTasks, &prvTask, &curTask)){
      printk_warn("Current task [0x%x] not registered \n", userTask);
      status =-1;
    }else{
      curPrc = curTask->trgtPrc;
      // Disable order_tasklet to prevent raceAround on order
      tasklet_disable(&(drvdata->order_tasklet));

      if(findOrder((waitOrder.task_id & NOC_TASKMASK), (waitOrder.from_id & NOC_TASKMASK), waitOrder.order_type,
                   curPrc->orderPool, &prvOrd, &curOrd)){ //order already received
        // send to user
        if(copy_to_user((void *) waitOrder.data,
                        (void *) curOrd->po, sizeof(struct orderHeader))){return -EFAULT;}
        // clear entry
        if (NULL == prvOrd){ //elmnt in head
          curPrc->orderPool = curOrd->nxtOrder;
        }else{
          prvOrd->nxtOrder = curOrd->nxtOrder;
        }
        kfree(curOrd);
        //enable tasklet and return
        tasklet_enable(&(drvdata->order_tasklet));
      }else{ // create entry and wait for order
        if(NULL == curOrd){ //list empty
          curPrc->orderPool = kmalloc(sizeof(oRcv), GFP_KERNEL);
          curOrd = curPrc->orderPool;
        }else{
          curOrd->nxtOrder = kmalloc(sizeof(oRcv), GFP_KERNEL);
          curOrd = curOrd->nxtOrder;
        }
        curOrd->task_id = (waitOrder.task_id & NOC_TASKMASK);
        curOrd->from_id = (waitOrder.from_id & NOC_TASKMASK);
        curOrd->order_type = waitOrder.order_type;
        curOrd->po = NULL;
        curOrd->nxtOrder = NULL;
        printk_HDBG("initiate a sleep for task 0x%x on prc 0x%x\n",
                    curOrd->task_id, curPrc->prc_id);
        //enable tasklet and sleep
        tasklet_enable(&(drvdata->order_tasklet));
        mutex_unlock(&drvdata->nocIpc_mutex);
        wait_event_interruptible(curPrc->prcQueue, 0x00 == curOrd->order_type);

        //return of sleep
        mutex_lock_interruptible(&drvdata->nocIpc_mutex);
        //disable tasklet before accessing orderPool
        tasklet_disable(&(drvdata->order_tasklet));
        if(copy_to_user((void *) waitOrder.data,
                        (void *) curOrd->po, sizeof(struct orderHeader))){return -EFAULT;}
        //delete order data and update pool struct
        kfree(curOrd->po);
        //updt prv ptr
        findOrder((waitOrder.task_id & NOC_TASKMASK), (waitOrder.from_id & NOC_TASKMASK), 0x00, curPrc->orderPool, &prvOrd, &curOrd);
        if (NULL == prvOrd){ //elmnt in head
          curPrc->orderPool = curOrd->nxtOrder;
        }else{
          prvOrd->nxtOrder = curOrd->nxtOrder;
        }
        kfree(curOrd);
        //enable tasklet and return
        tasklet_enable(&(drvdata->order_tasklet));
      }
    }
    break;
  };
  case NOC_GETPOWER:{
    u32 logBuffer[2];

    while(0x00 != readb(_RNOC(CMD))); // wait Noc rdy
    writeb(CNOC_NOCPOWER, _RNOC(CMD));
    while(0x00 != readb(_RNOC(CMD))); // wait Noc rdy
    logBuffer[0]= readl(_RNOC(SENDDATA));
    logBuffer[1]= readl(_RNOC(SENDDATA)+sizeof(u32));
    if(copy_to_user((void *) ioctl_arg, (void *)logBuffer, 2*sizeof(u32))){return -EFAULT;}

    break;
  };
  }

  mutex_unlock(&drvdata->nocIpc_mutex);
  return status;
}

/* ---
 * HANDLER FOR IRQ MANAGEMENT
 * ------------------------------------------------------ */
/* IRQ need two search in driver struct, allocate memory and
 * send data to userspace.
 * To keep Irq fast, work was divided in topHalf (isr) and
 * bottomHalf (tasklets).
 */
static void _order_tasklet(struct nocIpc_drvdata *drvdata){
  // read header value from fifo
  struct orderHeader *orderBuffer;
  tReg *curTask, *prvTask;
  pReg *curPrc;
  oRcv *curOrd, *prvOrd;
  u32 task_id, from_id;

  orderBuffer = kmalloc(sizeof(struct orderHeader), GFP_KERNEL);
  while(SORDERH_u32 == kfifo_out(&(drvdata->orderFifo), (u32*)orderBuffer, SORDERH_u32)){ //read all pending order
    printk_HDBG("Order read in \n");
    printk_HDBG("is_sync_header 0x%x\n", orderBuffer->is_sync_header);
    printk_HDBG("task_id 0x%x\n", orderBuffer->task_id);
    printk_HDBG("cmd 0x%x\n", orderBuffer->cmd);
    printk_HDBG("local 0x%lx\n", (ulong)orderBuffer->local);
    printk_HDBG("remote 0x%lx\n", (ulong)orderBuffer->remote);
    printk_HDBG("data_length 0x%x\n", orderBuffer->data_length);

    if(NOC_JOBASK == orderBuffer->cmd){
      task_id = MGMT_TASKID;
      from_id = 0x00;
    }else{
      task_id = (orderBuffer->task_id & NOC_TASKMASK);
      from_id = (orderBuffer->sender_id & NOC_TASKMASK);
    }

    //find trgtPrc
    if( !findTask(task_id, drvdata->regTasks, &prvTask, &curTask)){
      printk_warn("Target task [0x%x] not registered, order dropped.\n", task_id);
    }else{
      curPrc = curTask->trgtPrc;
      if(findOrder(task_id, from_id, orderBuffer->cmd,
                   curPrc->orderPool, &prvOrd, &curOrd)){ // prc waiting on order
                                // or another order of same kind already received
        if(NULL == curOrd->po){//prc waiting
          curOrd->po = orderBuffer;
          curOrd->order_type = 0x00; //setup wakeup flags
          printk_HDBG("initiate a wakeUp for task 0x%x on prc 0x%x\n",
                      task_id, curPrc->prc_id);
          tasklet_schedule(&drvdata->order_tasklet);//reschedule itsefl
          wake_up_interruptible(&curPrc->prcQueue);
          return;
        }else{//another order of same type already arrived
          while(NULL != curOrd->nxtOrder){ //find end
              curOrd = curOrd->nxtOrder;
          }
          goto appendNO;
        }
      }else{ //save it for later usage
          if(NULL == curOrd){ //list empty
            curPrc->orderPool = kmalloc(sizeof(oRcv), GFP_KERNEL);
            curOrd = curPrc->orderPool;
          }else{
          appendNO:
            curOrd->nxtOrder = kmalloc(sizeof(oRcv), GFP_KERNEL);
            curOrd = curOrd->nxtOrder;
          }
          curOrd->task_id = task_id;
          curOrd->from_id = from_id;
          curOrd->order_type = orderBuffer->cmd;
          curOrd->po = orderBuffer;
          curOrd->nxtOrder = NULL;
          printk_HDBG("Save order cmd 0x%x for task 0x%x on prc 0x%x\n",
                      curOrd->order_type, orderBuffer->task_id, curPrc->prc_id);
          //realloc for next read
          orderBuffer = kmalloc(sizeof(struct orderHeader), GFP_KERNEL);
      }
    }
  }
  kfree(orderBuffer);
  return;
}

static void _ack_tasklet(struct nocIpc_drvdata *drvdata){
  // read wait value from fifo
  struct waitPoint *waitpoint;
  tReg *curTask, *prvTask;
  pReg *curPrc;

  waitpoint = kmalloc(sizeof(struct waitPoint), GFP_KERNEL);
  while(kfifo_out(&(drvdata->ackFifo), waitpoint,1)){
    //find trgtPrc
    if( !findTask((waitpoint->task_id & NOC_TASKMASK), drvdata->regTasks, &prvTask, &curTask)){
      printk_err("Target task [0x%x] not registered, waitPoint Failed.\n", (waitpoint->task_id & NOC_TASKMASK));
    }else{
      curPrc = curTask->trgtPrc;
      //set wakeup flag
      curPrc->waitState.status = PRC_WAKEUP;
      curPrc->waitState.wp = waitpoint;
      tasklet_schedule(&drvdata->ack_tasklet);//reschedule itsefl
      wake_up_interruptible(&curPrc->prcQueue);
      return;
    }
  }
  kfree(waitpoint);
  return;
}

static irqreturn_t noc_isr_order(int irq, void *dev_id, struct pt_regs *regs)
{
  u8 i;
  struct orderHeader order;
  struct nocIpc_drvdata *drvdata = dev_id;
  if(irq != drvdata->irq_num[1]){ // guard
    return IRQ_NONE;
  }else{ // read nocItf register
    printk_HDBG("ORDER Interrupt generated\n");
    /* Acknoledge the interrupts by reading header in nocItf */
    while(0x00 != readb(_RNOC(CMD))); // wait Noc rdy
    writeb(CNOC_GORDER, _RNOC(CMD));
    while(0x00 != readb(_RNOC(CMD)));

    for( i=0; i< sizeof(struct orderHeader); i+=4){
      *((u32*)((void *)&order+i)) = readl(_RNOC(SENDDATA) + i);
    }
    if(!(order.is_sync_header)){
        printk_HDBG("Order read in \n");
        printk_HDBG("is_sync_header 0x%x\n", order.is_sync_header);
        printk_HDBG("task_id 0x%x\n", order.task_id);
        printk_HDBG("cmd 0x%x\n", order.cmd);
        printk_HDBG("local 0x%lx\n", (ulong)order.local);
        printk_HDBG("remote 0x%lx\n", (ulong)order.remote);
        printk_HDBG("data_length 0x%x\n", order.data_length);

        //Put value into FIFO.
        //NB: only one writer => not lock mechanism needed
        if(!(kfifo_in(&(drvdata->orderFifo), (u32*)&order, SORDERH_u32))){
          printk_warn("Order Fifo full: one order was drop off\n");
        }else{
          //Schedule bottom half in workqueue
          tasklet_schedule(&drvdata->order_tasklet);
        }
      }else{
      printk_HDBG("Interrupt generated but order list is empty [%d]\n", readb(_RNOC(RCVORDER)));
      }
      return IRQ_HANDLED;
      }
  }
static irqreturn_t noc_isr_wakeUp(int irq, void *dev_id, struct pt_regs *regs)
{
  u8 cb;
  struct waitPoint waitpoint;
  struct nocIpc_drvdata *drvdata = dev_id;
  if(irq != drvdata->irq_num[0]){ // guard
    return IRQ_NONE;
  }else{ // read nocItf register
    printk_HDBG("WakeUp Interrupt generated\n");
    /* Acknoledge the interrupts by reading wakeUp in nocItf */
    while(0x00 != readb(_RNOC(CMD))); // wait Noc rdy
    writeb(CNOC_WAKEUP, _RNOC(CMD));
    while(0x00 != readb(_RNOC(CMD)));

    for( cb=0; cb< sizeof(struct waitPoint); cb+=4){
      /* printk_dbg("read iteration %d => %lx\n",cb, RNOC_SENDDATA + cb); */
      *((u32*)((void *)&waitpoint+cb)) = readl(_RNOC(SENDDATA) + cb);
    }
    if(0x00 != waitpoint.task_id){
    printk_HDBG("WaitPoint property\n");
    printk_HDBG("task_id 0x%x\n", waitpoint.task_id);
    printk_HDBG("ack_type 0x%x\n", waitpoint.ack_type);

      //Put value into FIFO.
      //NB: only one writer => not lock mechanism needed
      if(!(kfifo_in(&(drvdata->ackFifo), &waitpoint, 1))){
        printk_warn("ack Fifo full: one ack was drop off\n");
      }else{
        //Schedule bottom half in workqueue
        tasklet_schedule(&drvdata->ack_tasklet);
      }
    }else{
      printk_HDBG("Interrupt generated but wakeUp list is empty [%d]\n", readb(_RNOC(WAKEUP)));
    }
    return IRQ_HANDLED;
  }
}
