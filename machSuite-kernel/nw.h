/**
 * @file nw.h
 * @brief Header file for emu-hmpsoc NW function.
 */
#ifndef _NW_IP
#define _NW_IP
#include <stdio.h>
#include <stdlib.h>

#define ALEN 128
#define BLEN 128

#define MATCH_SCORE 1
#define MISMATCH_SCORE -1
#define GAP_SCORE -1

#define ALIGN '\\'
#define SKIPA '^'
#define SKIPB '<'

#define MAX(A,B) ( ((A)>(B))?(A):(B) )

void needwun(char SEQA[ALEN], char SEQB[BLEN],
             char alignedA[ALEN+BLEN], char alignedB[ALEN+BLEN],
             int M[(ALEN+1)*(BLEN+1)], char ptr[(ALEN+1)*(BLEN+1)]);

#endif /*_NW_H*/
