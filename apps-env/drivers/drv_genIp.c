/* ----------------------------------------------------------------------------
 * Generic linux driver for machSuite Hardware compute stub
 * Author: baptiste.roux AT inria.fr
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

#include "genIp_utils.h"

#define  DRIVER_NAME "gen_ip"
#define MAX_DEVICES 20

//helper for accessing IP register
#define _RIP(NAME) (((void *)drvdata->ip_ptr+ (REG_##NAME)))

MODULE_LICENSE("GPL");
MODULE_AUTHOR("baptiste.roux AT inria.fr");
MODULE_DESCRIPTION("Generic driver for machsuite HW computation stub");
MODULE_VERSION("1.0");

/* ---
 * MODULE PARAMETERS
 * ------------------------------------------------------ */
static bool debug = false; /* print extra debug info */
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "enable debug info (default: false)");

#define printk_dbg(format, arg...) { if (debug) pr_info( "[PID 0x%x]=> " DRIVER_NAME ": %s:"  format, current->pid, __FUNCTION__, ## arg); }
#define printk_err(format, arg...)   pr_err("[PID 0x%x]=> " DRIVER_NAME ":"  format, current->pid, ## arg)
#define printk_info(format, arg...) pr_info("[PID 0x%x]=> " DRIVER_NAME ":"  format, current->pid, ## arg)
#define printk_warn(format, arg...) pr_warn("[PID 0x%x]=> " DRIVER_NAME ":"  format, current->pid, ## arg)
/* ---
 * HELP MESSAGE DISPLAY ON READ
 * -------------------------------------------------------------------------- */
const char help_read[]="Use ioctl cmd to use this generic IP manager\n\
                          IP_INIT    \n\
                          IP_START   \n\
                          IP_STATUS  \n\0";
static size_t data_read;


/* ---
 * GLOBAL DEFINE
 * ------------------------------------------------------ */
static struct class *ip_class;
static dev_t device_devt;
static int cur_devno=0;
static const struct of_device_id ip_of_match[] = {
  { .compatible = "emu-hmpsoc,genIp-1.0", },
  {}
};
MODULE_DEVICE_TABLE(of, ip_of_match);

struct ip_drvdata {
  u8 * dev_name;//UTILITY
  u32 major; u32 minor;
  struct cdev cdev;//CHRDEV REGISTER struct
  struct device *dev;
  u32 cursor;

  struct mutex ip_mutex;
  phys_addr_t ip_reg;
  resource_size_t ip_regsize;
  void __iomem *ip_ptr;

  u32 nbParams;
  u32 nbArrayIn;
  u32 nbArrayOut;

  //topHalf to bottomHalf IRQ communication
  u32 irq_num;
  bool irq_occur;
  wait_queue_head_t irq_wait;
  };

/* ---
 * Driver function prototypes and registration in kernel
 * file structure
 * ----------------------------------------------------- */
static int ip_open (struct inode *inode, struct file *file);
static int ip_release (struct inode *inode, struct file *file);
static ssize_t ip_read (struct file *file, char __user *user_buf, size_t size, loff_t *offset);
static ssize_t ip_write (struct file *, const char __user *user_buf, size_t size, loff_t *offset);
static long ip_ioctl(struct file *, unsigned int, unsigned long);
static irqreturn_t ip_isr(int irq, void *dev_id, struct pt_regs *regs);

static struct file_operations fops = {
  .open = ip_open,
  .release = ip_release,
  .read = ip_read,
  .write = ip_write,
  .compat_ioctl = ip_ioctl,
  .unlocked_ioctl = ip_ioctl
};


/* ---
 * Probing function prototypes and registration for
 * platform devices
 * ----------------------------------------------------- */
static int ip_probe(struct platform_device *pdev);
static int ip_remove(struct platform_device *pdev);

static struct platform_driver platform_driver = {
  .probe = ip_probe,
  .remove = ip_remove,
  .driver = {
    .name = DRIVER_NAME,
    .of_match_table = ip_of_match,
  },
};

static int ip_probe(struct platform_device *pdev)
{
  struct ip_drvdata * drvdata;
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
  match = of_match_device(ip_of_match, &pdev->dev);
  if (!match)
    {
      printk_err("Device doesn't match driver compatible\n");
      retval = -EINVAL;
      goto exit_0;
    }

  /*allocate drvdata memory */
  drvdata = kzalloc(sizeof(struct ip_drvdata), GFP_KERNEL);
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
  drvdata->ip_reg = res.start;
  drvdata->ip_regsize =resource_size(&res);

  /* read dtb irq info */
  drvdata->irq_num= irq_of_parse_and_map(pdev->dev.of_node, 0);
  if( !(drvdata->irq_num)){
    printk_err("Can't access irq value\n");
    retval = -1;
    goto exit_0;
  }
  if (of_property_read_u32(pdev->dev.of_node, "nb-params", &drvdata->nbParams)){
    printk_err("Couldn't determine nb-params information\n");
    goto exit_1;
  }
  if (of_property_read_u32(pdev->dev.of_node, "nb-arrayIn", &drvdata->nbArrayIn)){
    printk_err("Couldn't determine nb-arrayOut information\n");
    goto exit_1;
  }
  if (of_property_read_u32(pdev->dev.of_node, "nb-arrayOut", &drvdata->nbArrayOut)){
    printk_err("Couldn't determine nb-arrayOut information\n");
    goto exit_1;
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
  if (!request_mem_region(drvdata->ip_reg, drvdata->ip_regsize, drvdata->dev_name)) {
    printk_err("Can't request memory\n");
    retval = -ENOMEM;
    goto exit_1;
  }
  drvdata->ip_ptr = of_iomap(pdev->dev.of_node, 0);
  if(!(drvdata->ip_ptr)){
    printk_err("Can't do iomap\n");
    retval = -EFAULT;
    goto exit_2;
  }

  /*Ask for irq */
  retval = request_irq(drvdata->irq_num, (irq_handler_t) ip_isr,
                       IRQF_NOBALANCING, drvdata->dev_name, drvdata);
                       /* IRQF_SHARED, drvdata->dev_name, drvdata); */
  if(retval){
    printk_err("Can't request irq\n");
    goto exit_3;
  }

  /* Initialize device vars */
  drvdata->irq_occur = false;
  init_waitqueue_head(&(drvdata->irq_wait));
  mutex_init(&drvdata->ip_mutex);

  /*dump value*/
  printk_dbg("dev-name: %s \n", drvdata->dev_name);
  printk_dbg("register values read are: 0x%x  0x%x \n",
             drvdata->ip_reg, drvdata->ip_regsize);

  device_create(ip_class, &pdev->dev, devt, NULL, "%s",drvdata->dev_name);
  return retval;

 exit_3:
  free_irq(drvdata->irq_num, &pdev->dev);
 exit_2:
  iounmap(drvdata->ip_ptr);
 exit_1:
  vfree(drvdata->dev_name);
 exit_0:
  kfree(drvdata);
  return retval;
}

static int ip_remove(struct platform_device *pdev)
{
  struct ip_drvdata *drvdata;

  printk_dbg("Call remove function\n");

  drvdata = dev_get_drvdata(&pdev->dev);
  if(!drvdata)
    return 0;

  /*unregister device */
  mutex_destroy(&drvdata->ip_mutex);
  cdev_del(&drvdata->cdev);
  iounmap(drvdata->ip_ptr);
  release_mem_region(drvdata->ip_reg, drvdata->ip_regsize);
  free_irq(drvdata->irq_num, drvdata);
  vfree(drvdata->dev_name);
  kfree(drvdata);
  return 0;
}

/* ---
 * Init and Exit driver function
 * ------------------------------------------------------ */
static int __init ip_init(void)
{
  int retval = 0;

  printk_info("Register platform device \n");

  ip_class = class_create(THIS_MODULE, "ip_drv");
  ip_class->dev_groups = NULL;
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

static void __exit ip_exit(void)
{
  int i=0;
  dev_t devt;

  printk_info("UnRegister platform device \n");

  for(i=0 ; i<cur_devno ; i++){
    devt = MKDEV(MAJOR(device_devt), MINOR(device_devt)+i);
    device_destroy(ip_class, devt);
  }
  class_destroy(ip_class);
  platform_driver_unregister(&platform_driver);
  unregister_chrdev_region(device_devt, MAX_DEVICES);


}
module_init(ip_init);
module_exit(ip_exit);


/* ---
 * Driver function implementation
 * ----------------------------------------------------- */
static int ip_open (struct inode *inode, struct file *file)
{
  struct ip_drvdata *drvdata;
  drvdata = container_of(inode->i_cdev, struct ip_drvdata, cdev);
  file->private_data = drvdata;

  printk_dbg("OPEN DEVICE \n");
  try_module_get(THIS_MODULE);
  mutex_lock_interruptible(&drvdata->ip_mutex);
  drvdata->cursor = 0;//set read/Write cursor to 0
  return 0;
}

static int ip_release (struct inode *inode, struct file *file)
{
  struct ip_drvdata *drvdata = file->private_data;

  printk_dbg("RELEASE DEVICE \n");
  mutex_unlock(&drvdata->ip_mutex);
  module_put(THIS_MODULE);
  return 0;
}

static ssize_t ip_read (struct file *file, char __user *user_buf, size_t size, loff_t *offset)
{
  struct ip_drvdata *drvdata = file->private_data;
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

static ssize_t ip_write (struct file *file, const char __user *user_buf, size_t size, loff_t *offset)
{
  printk_info("No writing method implemented, use ioctl instead.\n");
  return -1;
}


static long ip_ioctl(struct file *file, unsigned int ip_cmd, unsigned long ioctl_arg)
{
  struct ip_drvdata *drvdata = file->private_data;

  u16 i;
  s16 status=0;

  printk_dbg("cmd=%d\n", ip_cmd);
  switch(ip_cmd){
  case IP_INIT:{
      /** @brief retrieve ioctl_arg from users
       * @params usr[0..nbParams]: (u32) params
       * @params usr[..nbArrayIn]: (u32) @ of input array
       * @params usr[..nbArrayOut]: (u32) @ of output array
       */
      u32 user_arg[drvdata->nbParams + drvdata->nbArrayIn + drvdata->nbArrayOut];
      if(copy_from_user((void *)&user_arg, (void *) ioctl_arg,
                        (drvdata->nbParams + drvdata->nbArrayIn + drvdata->nbArrayOut)*sizeof(u32)))
        {return -EFAULT;}
      for (i=0; i< drvdata->nbParams; i++){ // write parameters into ip
        writel(user_arg[i], _RIP(PARAMS)+ i*sizeof(u32));
      }
      for (i=0; i< drvdata->nbArrayIn; i++){ // write input array @ into ip
        writel(user_arg[i + drvdata->nbParams], _RIP(ARRAY_IN)+ i*sizeof(u32));
      }
      for (i=0; i< drvdata->nbArrayOut; i++){ // write output array @ into ip
        writel(user_arg[i + drvdata->nbParams + drvdata->nbArrayIn], _RIP(ARRAY_OUT)+ i*sizeof(u32));
      }
      break;
    };
  case IP_START:{
      writeb(0x80, _RIP(CMD)); // start ip
      wait_event_interruptible(drvdata->irq_wait, drvdata->irq_occur);
      drvdata->irq_occur = false;
      printk_dbg("Process wakeup after IRQ.\n");
      break;
    }
  case IP_STATUS:{
      u8 status;
      status = readb(_RIP(STATUS));
      if(copy_to_user((void *) ioctl_arg, (void *)&status, sizeof(u8))){return -EFAULT;}
      break;
    }
  }
  return status;
}

/* ---
 * HANDLER FOR IRQ MANAGEMENT
 * ------------------------------------------------------ */
static irqreturn_t ip_isr(int irq, void *dev_id, struct pt_regs *regs)
{
  struct ip_drvdata *drvdata = dev_id;
  if(irq != drvdata->irq_num){ // guard
    return IRQ_NONE;
  }else{
    drvdata->irq_occur = true;
    writeb(0x01, _RIP(CMD)); //ack irq0
    wake_up_interruptible(&drvdata->irq_wait);
    return IRQ_HANDLED;
  }
}
