/* ---
 *  Author <baptiste.roux AT inria.fr>
 * -------------------------------------------------------------------------- */
#ifndef __MON_ALLOC_H
#define __MON_ALLOC_H

/* ---
 *  Include
 * -------------------------------------------------------------------------- */
#include "drivers/cmd_monAlloc.h"
#include <stropts.h>
#include <stdio.h>
#include <malloc.h>

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <dlfcn.h>


#define DEV_MEM "/dev/mem"
#define DEV_SC "/dev/sc_memory"
#define MMAP_PROT            (PROT_READ|PROT_WRITE)
#define MMAP_FLAGS           (MAP_SHARED)

#ifdef MONALLOC_DEBUG
#   define Print_DBG(message)                             \
  do {std::cout << __PRETTY_FUNCTION__<<": "<< message<<"\n";}  \
  while (0)
#else
#   define Print_DBG(message)                   \
  do {} while (0)
#endif

/* ---
 *  Malloc hooks prototypes
 * -------------------------------------------------------------------------- */
/* This function is the user equivalent of the malloc subroutine. */
void *monitored_malloc(size_t);

/* This function is the user equivalent of the free subroutine.
 * work from real allocated blk or shared one
 */
void monitored_free(void *);

/* This function is help to retrieve SystemC physical addr from virtual one. */
uintptr_t monitored_VtoP(void * ptr);

/* This function is help to retrieve blk infos from virtual virtual addr to help blk sharing. */
void monitored_getBlkInfo(void * ptr, uintptr_t *blk_head, size_t *rOffset);

/* This function enable blk sharing between process */
void *monitored_shared(uintptr_t blk_head, size_t rOffset, size_t size);

/* This function is the user equivalent of the realloc subroutine. */
void *monitored_realloc(void *, size_t);

#endif /*__MON_ALLOC_H*/
