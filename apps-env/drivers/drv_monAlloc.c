/* ----------------------------------------------------------------------------
 * Linux driver for monitored Allocation on hMpSoC Virtual Platform
 * Author: baptiste.roux AT inria.fr
 *
 * This file implement memory blocks allocation in specified physical @range
 *     - Available memory blocks are retain in an chained list.
 *     - In-use blocks are retain in another chained-list with corresponding
 *       virtual address to ease the free process
 *
 * This driver use device tree facilities to probe available devices and then
 * implement basic file operation as char device driver
 * -------------------------------------------------------------------------- */

#include <linux/slab.h>/* linux header */
#include <linux/init.h>           // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>         // Core header for loading LKMs into the kernel
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <asm/uaccess.h>          // Required for the copy to user function
#include <linux/mutex.h>	  // Required for the mutex functionality
#include <linux/vmalloc.h>

#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/cdev.h>
#include <linux/sched.h>

#include "drv_monAlloc.h"
#include "cmd_monAlloc.h"


#define  DEVICE_NAME "monAlloc"    ///< The device will appear at /dev/ebbchar using this value
#define  DRIVER_NAME "drv_monAlloc"
#define  CLASS_NAME  "monAlloc"        ///< The device class -- this is a character device driver
#define MAX_DEVICES 10


MODULE_LICENSE("GPL");
MODULE_AUTHOR("baptiste.roux AT inria.fr");
MODULE_DESCRIPTION("memoryManager for monitored hmpsocvp memory");
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
const char help_read[]="Use ioctl cmd to use this memoryAllocation manager\n\
                        MONALLOC_CMD_ASK: ask for physical addr of available block\n\
                        MONALLOC_CMD_WASMAP: phys_addr map @virt_addr \n\
                        MONALLOC_CMD_RELEASE: release block previously map @virt_addr \n\
                        MONALLOC_CMD_STATUS: get driver status \n\0";
static size_t data_read;


/* ---
 * GLOBAL DEFINE
 * ------------------------------------------------------ */
static struct class *monAlloc_class;
static dev_t device_devt;
static int cur_devno=0;
static const struct of_device_id monAlloc_of_match[] = {
  { .compatible = "emu-hmpsoc,monAlloc-1.0", },
  {}
};
MODULE_DEVICE_TABLE(of, monAlloc_of_match);

struct monAlloc_drvdata {
  u8 * dev_name;//UTILITY
  u32 major; u32 minor;
  struct cdev cdev;//CHRDEV REGISTER struct
  struct device *dev;
  u32 cursor;

  struct mutex monAlloc_mutex;
  phys_addr_t mem_start;
  resource_size_t mem_size;
  resource_size_t chunk_size;
  memoryChunk *chunkPool;
  memoryChunk *avlblMem;
  memoryChunk *inUseMem;
  sharedChunk *sharedMem;
  u32 avlblChunks, inUseChunks;
  };

/* ---
 * Driver function prototypes and registration in kernel
 * file structure
 * ----------------------------------------------------- */
static int monAlloc_open (struct inode *inode, struct file *file);
static int monAlloc_release (struct inode *inode, struct file *file);
static ssize_t monAlloc_read (struct file *file, char __user *user_buf, size_t size, loff_t *offset);
static ssize_t monAlloc_write (struct file *, const char __user *user_buf, size_t size, loff_t *offset);
static long monAlloc_ioctl(struct file *, unsigned int, unsigned long);

static struct file_operations fops = {
  .open = monAlloc_open,
  .release = monAlloc_release,
  .read = monAlloc_read,
  .write = monAlloc_write,
  .compat_ioctl = monAlloc_ioctl,
  .unlocked_ioctl = monAlloc_ioctl
};


/* ---
 * Probing function prototypes and registration for
 * platform devices
 * ----------------------------------------------------- */
static int monAlloc_probe(struct platform_device *pdev);
static int monAlloc_remove(struct platform_device *pdev);

static struct platform_driver platform_driver = {
  .probe = monAlloc_probe,
  .remove = monAlloc_remove,
  .driver = {
    .name = DRIVER_NAME,
    .of_match_table = monAlloc_of_match,
  },
};

static int monAlloc_probe(struct platform_device *pdev)
{
  struct monAlloc_drvdata * drvdata;
  struct resource res;
  int retval =0;
  dev_t devt;
  /*specific dt property index*/
  const u8 *spec_prop_ptr;
  int spec_prop_len =0;
  int i ;

  printk_dbg("Call probing function\n");

  /*sanity check */
  const struct of_device_id *match;
  match = of_match_device(monAlloc_of_match, &pdev->dev);
  if (!match)
    {
      printk_err("Device doesn't match driver compatible\n");
      retval = -EINVAL;
      goto exit_0;
    }

  /*allocate drvdata memory */
  drvdata = kzalloc(sizeof(struct monAlloc_drvdata), GFP_KERNEL);
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
  drvdata->mem_start = res.start;
  drvdata->mem_size =resource_size(&res);
  if (of_property_read_u32(pdev->dev.of_node, "chunk-size", &drvdata->chunk_size)){
    printk_warn("Couldn't determine chunk-size information, set to default values");
    drvdata->chunk_size = 0x1000;
  }


  /* read dev-name info */
  spec_prop_ptr = of_get_property(pdev->dev.of_node, "dev-name",&spec_prop_len);
  if( NULL == spec_prop_ptr){
    printk_warn("Couldn't determine dev_name information, set to default values");
    drvdata->dev_name = vmalloc(6*sizeof(u8));
    drvdata->dev_name[0]='s'; drvdata->dev_name[1]='c'; drvdata->dev_name[2]='m';
    drvdata->dev_name[3]='e'; drvdata->dev_name[4]='m';drvdata->dev_name[5]='\0';
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

  /* Initialize device vars */
  drvdata->avlblChunks = (drvdata->mem_size/drvdata->chunk_size);
  if (0 != drvdata->avlblChunks){
    //Note: no need to release memoryChunks individually
    //    => allocate them as a big pool
    drvdata->chunkPool = vmalloc(drvdata->avlblChunks*sizeof(memoryChunk));
    drvdata->avlblMem = drvdata->chunkPool;
    memoryChunk* endOfAvlbl = drvdata->avlblMem;
    phys_addr_t nxtOffset = 0x00;
    while(nxtOffset < (drvdata->mem_size - drvdata->chunk_size)){
      endOfAvlbl->phys_addr =(drvdata->mem_start + nxtOffset);
      endOfAvlbl->virt_map_addr = 0x00; //only used when inUse
      endOfAvlbl->pendingShare = 0x00;
      endOfAvlbl->sharing = 0x00;
      endOfAvlbl->nxtChunk = (endOfAvlbl+1); //contiguous in virtMemory @creation
      nxtOffset +=drvdata->chunk_size;
      endOfAvlbl = endOfAvlbl->nxtChunk;
    }
    //init last slot
    endOfAvlbl->phys_addr = (drvdata->mem_start + drvdata->mem_size - drvdata->chunk_size);
    endOfAvlbl->virt_map_addr = 0x00;
    endOfAvlbl->pendingShare = 0x00;
    endOfAvlbl->sharing = 0x00;
    endOfAvlbl->nxtChunk = NULL;//contiguous in virtMemory @creation
  }else{
    printk_err("memory size to small to allocate memory chunks");
  }
  mutex_init(&drvdata->monAlloc_mutex);
  drvdata->inUseMem = NULL;
  drvdata->inUseChunks = 0;
  drvdata->sharedMem = NULL;

  /*dump value*/
  printk_dbg("dev-name: %s \n", drvdata->dev_name);
  printk_dbg("register values read are: 0x%x  0x%x \n",
             drvdata->mem_start, drvdata->mem_size);
  printk_dbg("Chunk_size: 0x%x\n", drvdata->chunk_size);

  device_create(monAlloc_class, &pdev->dev, devt, NULL, "%s",drvdata->dev_name);
  return retval;

 exit_1:
  vfree(drvdata->dev_name);
 exit_0:
  kfree(drvdata);
  return retval;
}

static int monAlloc_remove(struct platform_device *pdev)
{
  struct monAlloc_drvdata *drvdata;

  printk_dbg("Call remove function\n");

  drvdata = dev_get_drvdata(&pdev->dev);
  if(!drvdata)
    return 0;

  /*unregister device */
  mutex_destroy(&drvdata->monAlloc_mutex);
  cdev_del(&drvdata->cdev);
  vfree(drvdata->dev_name);
  vfree(drvdata->chunkPool);
  kfree(drvdata);
  return 0;
}

/* ---
 * Init and Exit driver function
 * ------------------------------------------------------ */
static int __init monAlloc_init(void)
{
  int retval = 0;

  printk_info("Register platform device \n");

  monAlloc_class = class_create(THIS_MODULE, "monAlloc_drv");
  monAlloc_class->dev_groups = NULL;
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

static void __exit monAlloc_exit(void)
{
  int i=0;
  dev_t devt;

  printk_info("UnRegister platform device \n");

  for(i=0 ; i<cur_devno ; i++){
    devt = MKDEV(MAJOR(device_devt), MINOR(device_devt)+i);
    device_destroy(monAlloc_class, devt);
  }
  class_destroy(monAlloc_class);
  platform_driver_unregister(&platform_driver);
  unregister_chrdev_region(device_devt, MAX_DEVICES);


}
module_init(monAlloc_init);
module_exit(monAlloc_exit);


/* ---
 * Driver function implementation
 * ----------------------------------------------------- */
static int monAlloc_open (struct inode *inode, struct file *file)
{
  struct monAlloc_drvdata *drvdata;
  drvdata = container_of(inode->i_cdev, struct monAlloc_drvdata, cdev);
  file->private_data = drvdata;
  drvdata->cursor = 0;//set read/Write cursor to 0

  printk_dbg("OPEN DEVICE \n");

  mutex_lock_interruptible(&drvdata->monAlloc_mutex);
  try_module_get(THIS_MODULE);
  return 0;
}

static int monAlloc_release (struct inode *inode, struct file *file)
{
  struct monAlloc_drvdata *drvdata = file->private_data;

  printk_dbg("RELEASE DEVICE \n");

  mutex_unlock(&drvdata->monAlloc_mutex);
  module_put(THIS_MODULE);
  return 0;
}

static ssize_t monAlloc_read (struct file *file, char __user *user_buf, size_t size, loff_t *offset)
{
  struct monAlloc_drvdata *drvdata = file->private_data;
  size_t n;
  if(drvdata->cursor >= data_read) { //all data has been read i.e EOF
    if(copy_to_user(user_buf, "\0", 1)){return -EFAULT;}
    return 0;
  }
  n = data_read - drvdata->cursor; //remainder
  n = (size >= n ? n : size);
  if(copy_to_user(user_buf, help_read+(drvdata->cursor), n)){return -EFAULT;}
  if(copy_to_user(user_buf+n, "\0", 1)){return -EFAULT;}
  drvdata->cursor += n;

  return n; //bytes_read;
}

static ssize_t monAlloc_write (struct file *file, const char __user *user_buf, size_t size, loff_t *offset)
{
  printk_info("No writing method implemented, use ioctl instead.\n");
  return -1;
}


static long monAlloc_ioctl(struct file *file, unsigned int monAlloc_cmd, unsigned long ioctl_arg)
{
  struct monAlloc_drvdata *drvdata = file->private_data;

  s8 status=0;
  memoryChunk *curScope;
  sharedChunk *curShared;
  sharedChunk *prvShared;
  //static chunk navPointer
  static memoryChunk *lastFix;
  static memoryChunk *chainHead;
  static memoryChunk *chainQueue;
  static sharedChunk *sharedHead;

  //user communication buffer
  u32 userBuffer[4];

  //contiguous chunk counters
  u16 contChunks=0;
  u16 findCont=0;

  printk_dbg("cmd=%d\n", monAlloc_cmd);
  switch(monAlloc_cmd){
  case MONALLOC_CMD_ASK:{
    if(copy_from_user((void *) userBuffer, (void *) ioctl_arg, sizeof(u32))){return -EFAULT;}
    printk_dbg("Ask for allocation of size 0x%x\n", userBuffer[0]);
    //find contiguous set of chunk that could host size byte
    contChunks = (((userBuffer[0]%drvdata->chunk_size)!=0)?
                      ((userBuffer[0]/drvdata->chunk_size)+1):(userBuffer[0]/drvdata->chunk_size));
    //init navPointer
    lastFix = NULL;
    curScope = drvdata->avlblMem;
    while((contChunks -1) != findCont){ //search contiguous space
      if((NULL == curScope) || (NULL == curScope->nxtChunk)){
        printk_err("No space left for contiguous allocation of size 0x%x\n", userBuffer[0]);
        userBuffer[0] = 0x00;
        if(copy_to_user((void *) ioctl_arg, (void *)userBuffer, sizeof(u32))){return -EFAULT;}
        return (-1);
      } else if((curScope->phys_addr + drvdata->chunk_size)
                == (curScope->nxtChunk->phys_addr)){
        findCont++;
        curScope = curScope->nxtChunk;
      } else {
        lastFix = curScope;
        curScope = curScope->nxtChunk;
        findCont = 0;
      }
    }
    chainHead = (NULL == lastFix)?(drvdata->avlblMem):(lastFix->nxtChunk);
    chainQueue = curScope;

    //Contiguous chunk find return RSize and physAddr to user
    userBuffer[0] = (contChunks*drvdata->chunk_size);
    userBuffer[1] = chainHead->phys_addr;
    if(copy_to_user((void *) ioctl_arg, (void *)userBuffer, 2*sizeof(u32))){return -EFAULT;}
    break;
  };
  case MONALLOC_CMD_WASMAP:{
    if(copy_from_user((void *) userBuffer, (void *) ioctl_arg, 3*sizeof(u32))){return -EFAULT;}
    if((0== (userBuffer[0]%drvdata->chunk_size)) && (chainHead->phys_addr == userBuffer[1])){ //Sanity Check
      printk_dbg("Register Alloc : s@0x%x, off@0x%x, v@0x%x\n",
                 userBuffer[0], userBuffer[1], userBuffer[2]);
      //updt chain info and mv it from available to unused
      chainHead->virt_map_addr = userBuffer[2];
      chainHead->map_in_prc = current->pid;
      if(NULL == lastFix){ //contiguous chunk are in head of available
        drvdata->avlblMem = chainQueue->nxtChunk;
      }else{ // contiguous are in the middle
        lastFix->nxtChunk = chainQueue->nxtChunk;
      }
      //cut chain end link and append it into inUse pool
      chainQueue->nxtChunk =NULL;
      if (NULL == drvdata->inUseMem){ //list empty
        drvdata->inUseMem = chainHead;
      }else{
        curScope = drvdata->inUseMem;
        while(NULL != curScope->nxtChunk){//find end
          curScope = curScope->nxtChunk;
        }
        curScope->nxtChunk = chainHead;
      }
      //updt status
      drvdata->avlblChunks -=(userBuffer[0]/drvdata->chunk_size);
      drvdata->inUseChunks +=(userBuffer[0]/drvdata->chunk_size);
      printk_dbg("MEM status: availables %d, inUse %d. \n",drvdata->avlblChunks, drvdata->inUseChunks);
    }else{
      printk_err("Incoherent mapping information");
      status=-1;
    }
    break;
  };
  case MONALLOC_CMD_ASKFREE:{ //Virt@ should be aligned on memoryChunk for unshared blk
    // read virt_map addr from user:
    if(copy_from_user((void *)userBuffer, (void *) ioctl_arg, sizeof(u32))){return -EFAULT;}
    printk_dbg("Search for v@0x%x\n", userBuffer[0]);
    // init navPointer
    lastFix = NULL;
    curScope = drvdata->inUseMem;
    u16 toUnmap=1;
    //find mapping in allocated one or in shared one
    if( NULL == curScope){goto exit_askfreeError;}
    while((userBuffer[0] != curScope->virt_map_addr)
          || (current->pid != curScope->map_in_prc)){//Find it in inUseMem
      lastFix = curScope;
      curScope = curScope->nxtChunk;
      if( NULL == curScope){goto searchInShared;}
    }
    chainHead = curScope;
    userBuffer[2] = curScope->phys_addr - drvdata->mem_start; //transpose physAddr to systemC DMA space
    printk_dbg("Chunk found : p@0x%x, v@0x%x\n", curScope->phys_addr, curScope->virt_map_addr);

    //find end of allocation chain
    while((NULL != curScope->nxtChunk) &&(0x00 == curScope->nxtChunk->virt_map_addr)){
      curScope = curScope->nxtChunk;
      toUnmap++;
    }
    chainQueue = curScope;
    userBuffer[1] = toUnmap*drvdata->chunk_size;
    if(copy_to_user((void *) ioctl_arg, (void *)userBuffer, 3*sizeof(u32))){return -EFAULT;}
    break;

    searchInShared:
    curShared = drvdata->sharedMem;
    if( NULL == curShared){goto exit_askfreeError;}
    while((userBuffer[0] != curShared->virt_map_addr)
          || (current->pid != curShared->map_in_prc)){//Find it in sharedMem
      curShared = curShared->nxtShare;
      if( NULL == curShared){goto exit_askfreeError;}
    }
    userBuffer[2] = (curShared->trgtChunk->phys_addr + curShared->phys_offset) - drvdata->mem_start; //transpose physAddr to systemC DMA space
    userBuffer[1] = curShared->size;
    sharedHead = curShared;
    if(copy_to_user((void *) ioctl_arg, (void *)userBuffer, 3*sizeof(u32))){return -EFAULT;}
    printk_dbg("Chunk found in shared pool: p@0x%x, v@0x%x\n", sharedHead->trgtChunk->phys_addr, sharedHead->virt_map_addr);
    break;

    exit_askfreeError:
    printk_err("Release chunks unknow v@0x%x\n", userBuffer[0]);
    status =-1;
    break;
  };
  case MONALLOC_CMD_WASFREE:{
    // read virt_map addr from user:
    if(copy_from_user((void *)userBuffer, (void *) ioctl_arg, 2*sizeof(u32))){return -EFAULT;}
    printk_dbg("UnRegister Alloc : s@0x%x, v@0x%x\n", userBuffer[1], userBuffer[0]);

    if((0== (userBuffer[1]%drvdata->chunk_size)) && (chainHead->virt_map_addr == userBuffer[0])){ //Real alloc freed
      if((0<chainHead->sharing) || (0xff ==chainHead->pendingShare)){//sharing exist or pending: unregister pid link but keep it in inUseMem
        chainHead->map_in_prc = -1;
        printk_dbg("inUsedMem: Sharing exist only unmap\n");
        break;
      }else{ // no sharing: realease memChunk
        goto freedChunk;
      }
    }else if(sharedHead->virt_map_addr == userBuffer[0]){ //shared freed
      printk_dbg("SharedMem: remove from linked list\n");
      curScope = sharedHead->trgtChunk;

      //remove sharedChunk from sharing list
      prvShared = NULL;
      curShared = drvdata->sharedMem;
      while(((current->pid != curShared->map_in_prc)||(curShared->virt_map_addr != sharedHead->virt_map_addr))
            && (NULL != curShared->nxtShare)) {
        prvShared = curShared;
        curShared = curShared->nxtShare;
      }

      sharedHead->virt_map_addr = 0x00;
      if(NULL == prvShared){//remove in head
        drvdata->sharedMem = sharedHead->nxtShare;
        vfree(sharedHead);
      }else{
        prvShared->nxtShare = sharedHead->nxtShare;
        vfree(sharedHead);
      }
      //decrease sharing value in memChunk and release it if necessary
      curScope->sharing --;
      if((0 >= curScope->sharing) && (-1 == curScope->map_in_prc) && (curScope->pendingShare != 0xff)){
        goto searchChunk;
      }else{
        userBuffer[0] = 0x0; // no blk released
        if(copy_to_user((void *) ioctl_arg, (void *)userBuffer, sizeof(u32))){return -EFAULT;}
        break;
      }
    }else{
    exit_wasfreeError:
      printk_err("Incoherent unMapping information\n");
      status=-1;
      break;
    }

    searchChunk: // find previous chunk to safely remove the current
    printk_dbg("release a blk from sharing informations\n");
    chainHead = curScope;
    lastFix = NULL;
    curScope = drvdata->inUseMem;
    u16 toUnmap=1;
    if( NULL == curScope){goto exit_wasfreeError;}
    while(chainHead->phys_addr != curScope->phys_addr){
      lastFix = curScope;
      curScope = curScope->nxtChunk;
      if( NULL == curScope){goto exit_wasfreeError;}
    }
    //find end of allocation chain
    while((NULL != curScope->nxtChunk) &&(0x00 == curScope->nxtChunk->virt_map_addr)){
      curScope = curScope->nxtChunk;
      toUnmap++;
    }
    userBuffer[1] = toUnmap*drvdata->chunk_size;
    chainQueue = curScope;

    freedChunk: //have chunk infos => rdy to release it
    printk_dbg("Release chunk\n");
    // ClearInfo and Move chain from inUse to available
    chainHead->virt_map_addr = 0x00;
    chainHead->map_in_prc = 0x00;
    chainHead->pendingShare = 0x00;
    if(NULL == lastFix){ //freed chunk are in head of inUse
      drvdata->inUseMem = chainQueue->nxtChunk;
    }else{ // contiguous are in the middle
      lastFix->nxtChunk = chainQueue->nxtChunk;
    }
    //reinject in avlblMem with phys Addr order to prevent artificial segmentation
    if ((NULL == drvdata->avlblMem)
        || (drvdata->avlblMem->phys_addr > chainQueue->phys_addr)){ // insert in Head
      chainQueue->nxtChunk = drvdata->avlblMem;
      drvdata->avlblMem = chainHead;
    }else{ //find insert point
      curScope = drvdata->avlblMem;
      while((NULL != curScope->nxtChunk)
            && (curScope->nxtChunk->phys_addr < chainQueue->phys_addr)){
        curScope= curScope->nxtChunk;
      }
      chainQueue->nxtChunk = curScope->nxtChunk;
      curScope->nxtChunk = chainHead;
    }
    //updt status
    drvdata->avlblChunks +=(userBuffer[1]/drvdata->chunk_size);
    drvdata->inUseChunks -=(userBuffer[1]/drvdata->chunk_size);
    userBuffer[0] = 0xffff; //notifyblk release
    if(copy_to_user((void *) ioctl_arg, (void *)userBuffer, sizeof(u32))){return -EFAULT;}

    printk_dbg("MEM status: %d %d \n",drvdata->avlblChunks, drvdata->inUseChunks);
    break;
  };
  case MONALLOC_CMD_GETPHYS:{ //Virt@ unaligned on memory chunks
    // read virt_map addr from user:
    if(copy_from_user((void *)userBuffer, (void *) ioctl_arg, 2*sizeof(u32))){return -EFAULT;}
    printk_dbg("Search for physAddr v@0x%x\n", userBuffer[0]);
    u32 virtOffset = userBuffer[0]%drvdata->chunk_size;
    u32 virtBase = userBuffer[0] - virtOffset;
    //Find the block mapped to the nearest virtual addr
    u32 nearestVirt = 0;
    chainHead = NULL;
    curScope = drvdata->inUseMem;
    while(NULL != curScope){ // find nearest virtual address
      if(current->pid == curScope->map_in_prc){
        if((virtBase >= curScope->virt_map_addr) && (nearestVirt < curScope->virt_map_addr)){
          nearestVirt = curScope->virt_map_addr; // update neareast value
          chainHead = curScope;
        }
      }
      curScope = curScope->nxtChunk;
    }

    //Check if the found block could handle the chunk offset
    if (NULL != chainHead){
      u32 linkedChunks=1;
      curScope = chainHead;
      while((NULL != curScope->nxtChunk) &&(0x00 == curScope->nxtChunk->virt_map_addr)){
        curScope = curScope->nxtChunk;
        linkedChunks++;
      }
      u32 chunkOffset = (virtBase - nearestVirt)/drvdata->chunk_size;
      if(linkedChunks > chunkOffset){
        if(0xffff == (userBuffer[1] & 0xffff)){//pending shared
          chainHead->pendingShare = 0xff;
        }
        userBuffer[1] = ((chainHead->phys_addr - drvdata->mem_start)
                         + chunkOffset*drvdata->chunk_size
                         + virtOffset); //Compute phys@ transposed in systemC DMA space
        userBuffer[2] = chainHead->phys_addr; //phys addr of blk head
        userBuffer[3] = ((chunkOffset*drvdata->chunk_size)+ virtOffset); //offset from blk head
        printk_dbg(" v@0x%x found in inUseMemory chunk base p@0x%x => sc@0x%x\n",
                   userBuffer[0], userBuffer[2], userBuffer[1] );
        if(copy_to_user((void *) ioctl_arg, (void *)userBuffer, 4*sizeof(u32))){return -EFAULT;}
        break;
      }
    }
    //If reach this point no valid block found inUseMem search in shared one
    nearestVirt = 0;
    sharedHead = NULL;
    curShared = drvdata->sharedMem;
    while(NULL != curShared){ // find nearest virtual address
      if(current->pid == curShared->map_in_prc){
        if((virtBase >= curShared->virt_map_addr) && (nearestVirt < curShared->virt_map_addr)){
          nearestVirt = curShared->virt_map_addr; // update neareast value
          sharedHead = curShared;
        }
      }
      curShared = curShared->nxtShare;
    }
    // Check if sharedBlk could handle the offset
    u32 virtBlkOffset = (virtBase - nearestVirt);
    if( NULL != sharedHead){
      if(sharedHead->size > virtOffset){
        if(0xffff == (userBuffer[1] & 0xffff)){//pending shared
          sharedHead->trgtChunk->pendingShare = 0xff;
        }
        userBuffer[1] = ((sharedHead->trgtChunk->phys_addr - drvdata->mem_start)
                         + sharedHead->phys_offset + virtBlkOffset
                         + virtOffset); //Compute phys@ transposed in systemC DMA space
        userBuffer[2] = sharedHead->trgtChunk->phys_addr;
        userBuffer[3] = sharedHead->phys_offset + virtBlkOffset + virtOffset;
        printk_dbg(" v@0x%x found in sharedMem chunk base p@0x%x => sc@0x%x\n",
                   userBuffer[0], userBuffer[2], userBuffer[1] );
        if(copy_to_user((void *) ioctl_arg, (void *)userBuffer, 4*sizeof(u32))){return -EFAULT;}
        break;
      }
    }

    //If reach this point no valid block found print error msg
    printk_err(" v@0x%x wasn't found in mapped memory chunk\n", userBuffer[0]);
    status =-1;
    break;
  };

  case MONALLOC_CMD_SHARED:{
    //Read users args: [0] head Phys blkAddr, [1] offset from head, [2] virt@ of subchunk, [3] size
    if(copy_from_user((void *)userBuffer, (void *) ioctl_arg, 4*sizeof(u32))){return -EFAULT;}
    printk_dbg(" Shared blk_head p@0x%x + offset 0x%x registered at v@0x%x [Len 0x%x]\n",
               userBuffer[0], userBuffer[1], userBuffer[2], userBuffer[3]);
    //Find blk in inUseMem and increased the shared flag
    curScope = drvdata->inUseMem;
    if( NULL == curScope){goto exit_sharedError;}
    while(userBuffer[0] != curScope->phys_addr){//Find blkHead inUseMem
      curScope = curScope->nxtChunk;
      if( NULL == curScope){goto exit_sharedError;}
    }
    curScope->pendingShare = 0x00; // reset pending flag
    curScope->sharing++; //increased sharing flag

    //Add  a slot in sharedBlk list
    curShared = drvdata->sharedMem;
    if(NULL == curShared){ //insert in head
      drvdata->sharedMem = (sharedChunk*) vmalloc(sizeof(sharedChunk));
      curShared = drvdata->sharedMem;
    }else{//find end
      while ( NULL != curShared->nxtShare){curShared = curShared->nxtShare;}
      curShared->nxtShare = (sharedChunk*) vmalloc(sizeof(sharedChunk));
      curShared = curShared->nxtShare;
    }
    curShared->phys_offset = userBuffer[1];
    curShared->virt_map_addr = userBuffer[2];
    curShared->size = userBuffer[3];
    curShared->map_in_prc = current->pid;
    curShared->trgtChunk = curScope;
    curShared->nxtShare = NULL;
    break;

    exit_sharedError:
    printk_err("Target sharing chunks unknow p@0x%x\n", userBuffer[0]);
    status =-1;
    break;
  };
  case MONALLOC_CMD_STATUS:{
    u32 buffer[3];
    buffer[0] =(drvdata->mem_size/drvdata->chunk_size);
    buffer[1] = drvdata->avlblChunks;
    buffer[2] = drvdata->inUseChunks;
    printk_dbg("Chunks in pool: %d\n \t availables: %d\n \t inUse: %d\n",
               buffer[0], buffer[1], buffer[2]);
    if(copy_to_user((void *) ioctl_arg, (void *)buffer, 3*sizeof(u32))){return -EFAULT;}
    break;
  };
  }

  /* DEBUG HELPER ----------------------------------------------------------- */
  /* memoryChunk *tmp=drvdata->avlblMem; */
  /* int guard=12; */
  /* printk_dbg("AVAILABLE MEMORY:\n"); */
  /* while((tmp !=NULL) && (0<guard--)){ */
  /*   printk_dbg("Chunk p@%x, v@%x\n", tmp->phys_addr, tmp->virt_map_addr); */
  /*   tmp = tmp->nxtChunk; */
  /* } */

  /* guard=12; */
  /* tmp = drvdata->inUseMem; */
  /* printk_dbg("INUSE MEMORY:\n"); */
  /* while((tmp !=NULL) && (0<guard--)){ */
  /*   printk_dbg("Chunk p@%x, v@%x\n", tmp->phys_addr, tmp->virt_map_addr); */
  /*   tmp = tmp->nxtChunk; */
  /* } */
  /* if (NULL != lastFix) */
  /*   printk_dbg("LastFIX => p@%x, v@%x\n", lastFix->phys_addr, lastFix->virt_map_addr); */
  /* if (NULL != chainHead) */
  /* printk_dbg("chainHead => p@%x, v@%x\n", chainHead->phys_addr, chainHead->virt_map_addr); */
  /* if (NULL != chainQueue) */
  /* printk_dbg("chainQueue => p@%x, v@%x\n", chainQueue->phys_addr, chainQueue->virt_map_addr); */
  /* if (NULL != chainQueue->nxtChunk) */
  /*   printk_dbg("nxtFix => p@%x, v@%x\n", chainQueue->nxtChunk->phys_addr, chainQueue->nxtChunk->virt_map_addr); */

  /* guard=12; */
  /* sharedChunk *tsh = drvdata->sharedMem; */
  /* printk_dbg("shared MEMORY:\n"); */
  /* while((tsh !=NULL) && (0<guard--)){ */
  /*   printk_dbg("Shared Chunk p@0x%x + 0x%x, v@%x in 0x%x\n", tsh->trgtChunk->phys_addr, tsh->phys_offset, tsh->virt_map_addr, tsh->map_in_prc); */
  /*   tsh = tsh->nxtShare; */
  /* } */
  /* END DEBUG HELPER ------------------------------------------------------- */
  return status;
}
