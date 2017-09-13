/**
 * @file mutantExec.h
 * @brief Header file for mutantGraph execution application
 */
#ifndef _MUTANTEXEC
#define _MUTANTEXEC

#ifdef MEXEC_DEBUG
#   define print_DMEXEC(message)                           \
  do {std::cout << message;}                              \
  while (false)
#define assert(cond, message) \
  do{ if(!(cond)){ std::cout<< __PRETTY_FUNCTION__<<"Assertion failure: "<< message<< std::endl;}} \
  while(false)
#else
#   define print_DMEXEC(message)                 \
  do {} while (false)
#define assert(cond, message)                                           \
  do{} while(false)
#endif


// Generic include
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <vector>

// Modules include
#include "mutantGen.h"
#include "clMgmtAPI.h"
extern "C" {
#include "monAlloc.h"
}

// TODO extract these info from noc properties
#define CL_BASE_ADDR 0x60000000
#define CL_OFFSET_ADDR 0x20000000

/// Some helper function

noc::task_t getTaskUID(uint task, bool hw, uint id);
std::vector<std::tuple<noc::task_t, size_t, uint64_t>> getProviders(mutantGen *graph, uint x_pos, uint y_pos);
std::vector<std::pair<noc::task_t, uint64_t>> getReceivers(mutantGen *graph, uint x_pos, uint y_pos);

#endif /*_MUTANTEXEC*/
