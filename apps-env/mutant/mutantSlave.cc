/*
 * Copyright (c) 2017 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: muntantSlave.cc
 * executable for slave clusters
 */
#include "clMgmtAPI.h"
extern "C" {
#include "monAlloc.h"
}

int main(int argc, char ** argv)
{
  clmgmt::clEndPoint ep("/dev/sc_noc");
  return(EXIT_SUCCESS);
}
