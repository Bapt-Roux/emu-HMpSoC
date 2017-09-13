/**
 * @file sort_merge.h
 * @brief Header file for emu-hmpsoc SORT_MERGE function.
 */
#ifndef _SORT_MERGE_IP
#define _SORT_MERGE_IP

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SIZE 2048
#define TYPE_SM int32_t
#define TYPE_SM_MAX INT32_MAX

void ms_mergesort(TYPE_SM a[SIZE]);
#endif /*_SORT_MERGE_H*/
