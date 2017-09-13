/**
 * @file machSuite_cls_helper.h
 * @brief help traduction between benchName and id
 * @author <baptiste.roux AT inria.fr>
 */
#ifndef _MACHSUITE_CLS_HELPER
#define _MACHSUITE_CLS_HELPER

/**
 * @brief constant definition of task id
 */
#define ID_AES 0x1
#define ID_VITERBI 0x2
#define ID_BFSBULK 0x3
#define ID_BACKPROP 0x4
#define ID_FFTSTRIDED 0x5
#define ID_GEMMBLOCKED 0x6
#define ID_KMP 0x7
#define ID_MDKNN 0x8
#define ID_NW 0x9
#define ID_SORTMERGE 0xa
#define ID_STENC2D 0xb
#define ID_SPMV 0xc


static const std::map<uint,std::string> task_helper = {{ID_AES, "Aes"}, {ID_VITERBI, "Viterbi"},
                                                       {ID_BFSBULK, "Bfs bulk"}, {ID_BACKPROP, "Backprop"},
                                                       {ID_FFTSTRIDED, "FFT strided"}, {ID_GEMMBLOCKED, "Gemm blocked"},
                                                       {ID_KMP, "Kmp"}, {ID_MDKNN, "Md"},
                                                       {ID_NW, "Nw"},{ID_SORTMERGE, "Sort merge"},
                                                       {ID_STENC2D, "Stencil"}, {ID_SPMV, "Spmv"}};

#endif /*_MACHSUITE_CLS_HELPER*/
