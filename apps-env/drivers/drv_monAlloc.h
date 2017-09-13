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

/* ---
 * PREPROCESSOR MACCROS
 * ------------------------------------------------------ */
#define printk_dbg(format, arg...) { if (debug) pr_info(DRIVER_NAME ": %s:"  format, __FUNCTION__, ## arg); }
#define printk_err(format, arg...) pr_err(DRIVER_NAME ":"  format, ## arg)
#define printk_info(format, arg...) pr_info(DRIVER_NAME ":"  format, ## arg)
#define printk_warn(format, arg...) pr_warn(DRIVER_NAME ":"  format, ## arg)

/* ---
 * DRIVER STRUCT
 * ------------------------------------------------------ */
typedef struct memChunk{
  phys_addr_t phys_addr;
  phys_addr_t virt_map_addr;
  pid_t map_in_prc;
  uint sharing;
  uint8_t pendingShare;
  struct memChunk *nxtChunk;
}memoryChunk;

typedef struct sh_Chunk{
  phys_addr_t phys_offset; //could be unaligned
  phys_addr_t virt_map_addr;
  pid_t map_in_prc;
  size_t size;
  struct memChunk *trgtChunk;
  struct sh_Chunk *nxtShare;
}sharedChunk;
