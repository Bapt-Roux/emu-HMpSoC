/* ---
 *  This file implement malloc and free hooks that enable the used of
 *  monitored systemC memory.
 *
 *  Author <baptiste.roux AT inria.fr>
 *
 *  Description: Do memory allocation and release in systemC memory through
                 driver system call.
 *  Dependancy: this file need following header
 *         #drv_monAlloc.ko: linux driver that manage the monitored memory pool
 *         #cmd_monAlloc.h: header contain command code
 *         #monAlloc.h: header with memory structure and function header
 * -------------------------------------------------------------------------- */
#include "monAlloc.h"
static int dev_mem_fd=-1;
static unsigned int cur_allocation=0;

void *monitored_malloc(size_t size){
#ifdef MONALLOC_DEBUG
  printf("%s: called with size %d \n Current state: fd %d, curAlloc %d\n",
         __PRETTY_FUNCTION__, size, dev_mem_fd, cur_allocation);
#endif
  int dev_scMem_fd;
  long buffer[3];
  void* ptr;

  buffer[0] = (long)size;

  dev_scMem_fd = open(DEV_SC, O_RDWR); //take driver mutex
  if(0>ioctl(dev_scMem_fd, MONALLOC_CMD_ASK, buffer)){close(dev_scMem_fd); return NULL;}
  if(dev_mem_fd < 0){
    dev_mem_fd = open(DEV_MEM, O_RDWR);
  }
  ptr = mmap(NULL, buffer[0], MMAP_PROT, MMAP_FLAGS, dev_mem_fd, buffer[1]);
  if(MAP_FAILED == ptr){
    perror("mmap failed");
    close(dev_scMem_fd); //release driver
    return(NULL);
  }else{
    cur_allocation++;
    buffer[2] = (long)ptr;
    if(0>ioctl(dev_scMem_fd, MONALLOC_CMD_WASMAP, buffer)){close(dev_scMem_fd); return NULL;}
    close(dev_scMem_fd); //release driver

#ifdef MONALLOC_DEBUG
  printf("%s: Allocate size 0x%lx, p@0x%lx, v@0x%lx\n",
         __PRETTY_FUNCTION__, buffer[0], buffer[1], buffer[2]);
#endif
  return(ptr);
  }
}


void monitored_free(void * ptr){
#ifdef MONALLOC_DEBUG
  printf("%s: called on ptr 0x%lx\n Current state: fd %d, curAlloc %d\n",
         __PRETTY_FUNCTION__, (long)ptr, dev_mem_fd, cur_allocation);
#endif
  int dev_scMem_fd;
  uint status=0;
  long buffer[2];

  buffer[0] = (long)ptr;
  dev_scMem_fd = open(DEV_SC, O_RDWR); //take driver mutex
  if(0>ioctl(dev_scMem_fd, MONALLOC_CMD_ASKFREE, buffer)){close(dev_scMem_fd);return;}
  munmap((void*)buffer[0],(size_t) buffer[1]);

  if(ioctl(dev_scMem_fd, MONALLOC_CMD_WASFREE, buffer)){close(dev_scMem_fd); return;}
  if (0xffff == buffer[0]){//a blk release occured
    cur_allocation--;
    if(0==cur_allocation){
      close(dev_mem_fd);
      dev_mem_fd =-1;
    }
  }
  close(dev_scMem_fd); //release driver
  return;
}


uintptr_t monitored_VtoP(void * ptr){
#ifdef MONALLOC_DEBUG
  printf("%s: called on ptr 0x%lx\n Current state: fd %d, curAlloc %d\n",
         __PRETTY_FUNCTION__, (long)ptr, dev_mem_fd, cur_allocation);
#endif
  int dev_scMem_fd;
  long buffer[4];

  buffer[0] = (long)ptr;
  buffer[1] = (long)0x0000; //no pending sharing
  dev_scMem_fd = open(DEV_SC, O_RDWR); //take driver mutex
  if(0>ioctl(dev_scMem_fd, MONALLOC_CMD_GETPHYS, buffer)){close(dev_scMem_fd); return;}
  close(dev_scMem_fd); //release driver
  return((uintptr_t)buffer[1]);
}

void monitored_getBlkInfo(void * ptr, uintptr_t *blk_head, size_t *rOffset){
#ifdef MONALLOC_DEBUG
  printf("%s: called on ptr 0x%lx\n Current state: fd %d, curAlloc %d\n",
         __PRETTY_FUNCTION__, (long)ptr, dev_mem_fd, cur_allocation);
#endif
  int dev_scMem_fd;
  long buffer[4];

  buffer[0] = (long)ptr;
  buffer[1] = (long)0xffff; //pending sharing
  dev_scMem_fd = open(DEV_SC, O_RDWR); //take driver mutex
  if(0>ioctl(dev_scMem_fd, MONALLOC_CMD_GETPHYS, buffer)){close(dev_scMem_fd); return;}
  close(dev_scMem_fd); //release driver
  //set outputs
  *blk_head = buffer[2];
  *rOffset = buffer[3];
  return;
}

void *monitored_shared(uintptr_t blk_head, size_t rOffset, size_t size){
#ifdef MONALLOC_DEBUG
  printf("%s: called with size %d \n Current state: fd %d, curAlloc %d\n",
         __PRETTY_FUNCTION__, size, dev_mem_fd, cur_allocation);
#endif
  int dev_scMem_fd;
  long buffer[4];
  void* ptr;

  //mmap part of the blk in user process
  if(dev_mem_fd < 0){dev_mem_fd = open(DEV_MEM, O_RDWR);}
  // aligned map for preventing failures, applied offset on v@
  size_t align = rOffset & 0xfff;
  ptr = (void*) mmap(NULL, size + align, MMAP_PROT, MMAP_FLAGS, dev_mem_fd, blk_head+ (rOffset-align));
  if(MAP_FAILED == ptr){
    perror("mmap failed in monitored_shared");
    return(NULL);
  }else{//register sharing in driver
    ptr = (void*) ((uintptr_t)ptr + align); // unaligned to point on data
    buffer[0] = blk_head;
    buffer[1] = rOffset;
    buffer[2] = (uintptr_t) ptr;
    buffer[3] = size + align;
    /* buffer[3] = size + rOffset; */
    dev_scMem_fd = open(DEV_SC, O_RDWR); //take driver mutex
    if(0>ioctl(dev_scMem_fd, MONALLOC_CMD_SHARED, buffer)){
      close(dev_scMem_fd);
      munmap((void*)ptr,(size_t) size);
      return NULL;
    }else{
      close(dev_scMem_fd); //release driver
      return (ptr);
    }
  }
}

void *monitored_realloc(void * ptr, size_t size){
#ifdef MONALLOC_DEBUG
  printf("%s: called on ptr 0x%lx\n Current state: fd %d, curAlloc %d\n",
         __PRETTY_FUNCTION__, (long)ptr, dev_mem_fd, cur_allocation);
#endif
  monitored_free(ptr);
  return (monitored_malloc(size));
}
