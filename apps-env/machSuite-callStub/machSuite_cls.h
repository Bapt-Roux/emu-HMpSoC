/**
 * @file machSuite_hws.h
 * @brief top level header for machsuite HW stub.
 * @author <baptiste.roux AT inria.fr>
 */
#ifndef _MACHSUITE_CLS
#define _MACHSUITE_CLS

//Force local cpu caching
// define LOCAL_CPU_BUFFER in compiler flags
//Access more predictable and faster (only one read/write from monitored memory)

#include <map>
#include "noc_helper.h"
#include "machSuite_cls_helper.h"
#include "aes_cls.h"
#include "viterbi_cls.h"
#include "bfs_bulk_cls.h"
#include "backprop_cls.h"
#include "fft_strided_cls.h"
#include "gemm_blocked_cls.h"
#include "kmp_cls.h"
#include "md_knn_cls.h"
#include "nw_cls.h"
#include "sort_merge_cls.h"
#include "stenc2d_cls.h"
#include "spmv_crs_cls.h"

/**
 * @brief functions map to instantiate a function code from a task ID.
 *     Function data are dynamically acquired with local/noc communication @start
 *         and written the same way at this end.
 * @params NOTE: Job_id are built as follow:
 *    [(2) TYPE (0:SW, 1,2,3:HW)] [6: BENCH]
 */

typedef int (*worker_func)(void * dataIn, void* &dataOut, bool &localAlloc);

const std::map<noc::task_t, worker_func> machSuiteWorker ={
  {ID_AES <<TASK_TO_TID, aes},
  {HWFLAG | ID_AES <<TASK_TO_TID, aes_hw},
  {ID_VITERBI <<TASK_TO_TID, viterbi},
  {HWFLAG | ID_VITERBI <<TASK_TO_TID, viterbi_hw},
  {ID_BFSBULK <<TASK_TO_TID, bfs_bulk},
  {HWFLAG | ID_BFSBULK <<TASK_TO_TID, bfs_bulk_hw},
  {ID_BACKPROP <<TASK_TO_TID, backprop},
  {HWFLAG | ID_BACKPROP <<TASK_TO_TID, backprop_hw},
  {ID_FFTSTRIDED <<TASK_TO_TID, fft_strided},
  {HWFLAG | ID_FFTSTRIDED <<TASK_TO_TID, fft_strided_hw},
  {ID_GEMMBLOCKED <<TASK_TO_TID, gemm_blocked},
  {HWFLAG | ID_GEMMBLOCKED <<TASK_TO_TID, gemm_blocked_hw},
  {ID_KMP <<TASK_TO_TID, kmp},
  {HWFLAG | ID_KMP <<TASK_TO_TID, kmp_hw},
  {ID_MDKNN <<TASK_TO_TID, md_knn},
  {HWFLAG | ID_MDKNN <<TASK_TO_TID, md_knn_hw},
  {ID_NW <<TASK_TO_TID, nw},
  {HWFLAG | ID_NW <<TASK_TO_TID, nw_hw},
  {ID_SORTMERGE <<TASK_TO_TID, sort_merge},
  {HWFLAG | ID_SORTMERGE <<TASK_TO_TID, sort_merge_hw},
  {ID_STENC2D <<TASK_TO_TID, stenc2d},
  {HWFLAG | ID_STENC2D <<TASK_TO_TID, stenc2d_hw},
  {ID_SPMV <<TASK_TO_TID, spmv_crs},
  {HWFLAG | ID_SPMV <<TASK_TO_TID, spmv_crs_hw}
};

/**
 * @brief functions map to instantiate a function code from a task ID.
 *     Contain data_generator functions.
 * @params iter: number of iterations
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 */

typedef void (*genData_func)(uint32_t iter, void* &in,size_t &lIn, void* &out, size_t &lOut);

const std::map<noc::task_t, genData_func> machSuiteGenData ={
  {ID_AES <<TASK_TO_TID, aes_genData},
  {ID_VITERBI <<TASK_TO_TID, viterbi_genData},
  {ID_BFSBULK <<TASK_TO_TID, bfs_bulk_genData},
  {ID_BACKPROP <<TASK_TO_TID, backprop_genData},
  {ID_FFTSTRIDED <<TASK_TO_TID, fft_strided_genData},
  {ID_GEMMBLOCKED <<TASK_TO_TID, gemm_blocked_genData},
  {ID_KMP <<TASK_TO_TID, kmp_genData},
  {ID_MDKNN <<TASK_TO_TID, md_knn_genData},
  {ID_NW <<TASK_TO_TID, nw_genData},
  {ID_SORTMERGE <<TASK_TO_TID, sort_merge_genData},
  {ID_STENC2D <<TASK_TO_TID, stenc2d_genData},
  {ID_SPMV <<TASK_TO_TID, spmv_crs_genData}
};

/**
 * @brief functions map to instantiate a function code from a task ID.
 *     Retrieve golden inputs/outputs samples
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params valid: uninitialized ptr to golden outputs
 */

typedef void (*getSamples_func)(void* &in,size_t &lIn, void* &out, size_t &lOut, void* &valid);

const std::map<noc::task_t, getSamples_func> machSuiteGetSamples ={
  {ID_AES <<TASK_TO_TID, aes_getSamples},
  {ID_VITERBI <<TASK_TO_TID, viterbi_getSamples},
  {ID_BFSBULK <<TASK_TO_TID, bfs_bulk_getSamples},
  {ID_BACKPROP <<TASK_TO_TID, backprop_getSamples},
  {ID_FFTSTRIDED <<TASK_TO_TID, fft_strided_getSamples},
  {ID_GEMMBLOCKED <<TASK_TO_TID, gemm_blocked_getSamples},
  {ID_KMP <<TASK_TO_TID, kmp_getSamples},
  {ID_MDKNN <<TASK_TO_TID, md_knn_getSamples},
  {ID_NW <<TASK_TO_TID, nw_getSamples},
  {ID_SORTMERGE <<TASK_TO_TID, sort_merge_getSamples},
  {ID_STENC2D <<TASK_TO_TID, stenc2d_getSamples},
  {ID_SPMV <<TASK_TO_TID, spmv_crs_getSamples}
};

/**
 * @brief functions map to instantiate a function code from a task ID.
 *     Retrieve golden inputs/outputs samples
 * @params in: uninitialized ptr to inputs data
 * @params lIn: length of the generate inputs
 * @params out: uninitialized ptr to outputs buffer
 * @params lOut: length of the generate outputs buffers
 * @params valid: uninitialized ptr to golden outputs
 */

typedef void (*getDataSize_func)(uint iter, size_t &lIn, size_t &lOut);

const std::map<noc::task_t, getDataSize_func> machSuiteDataSize ={
  {ID_AES <<TASK_TO_TID, aes_getDataSize},
  {ID_VITERBI <<TASK_TO_TID, viterbi_getDataSize},
  {ID_BFSBULK <<TASK_TO_TID, bfs_bulk_getDataSize},
  {ID_BACKPROP <<TASK_TO_TID, backprop_getDataSize},
  {ID_FFTSTRIDED <<TASK_TO_TID, fft_strided_getDataSize},
  {ID_GEMMBLOCKED <<TASK_TO_TID, gemm_blocked_getDataSize},
  {ID_KMP <<TASK_TO_TID, kmp_getDataSize},
  {ID_MDKNN <<TASK_TO_TID, md_knn_getDataSize},
  {ID_NW <<TASK_TO_TID, nw_getDataSize},
  {ID_SORTMERGE <<TASK_TO_TID, sort_merge_getDataSize},
  {ID_STENC2D <<TASK_TO_TID, stenc2d_getDataSize},
  {ID_SPMV <<TASK_TO_TID, spmv_crs_getDataSize}
};

#endif /*_MACHSUITE_CLS*/
