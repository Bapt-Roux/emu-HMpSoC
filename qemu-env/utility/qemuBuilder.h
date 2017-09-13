/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: qemuBuilder.h
 * Configuration utility to read socConfiguration and to thrown corresponding qemu instance
 * with the help of petalinux toolchains
 */

#ifndef _QEMU_BUILDER
#define _QEMU_BUILDER
#include "models/utility/socConfigStruct.h"
#include <unistd.h>
#include <sys/wait.h>


namespace qb{
  /*---
   * CLASS DEFINITION
   *----------------------------------------------------------------------------*/
  /*
   * qemuBuilder class
   * Provides facilities to thrown qemu instance following a SoC configuration
   * => Read configuration file, spawn qemu cluster process
   * => after starts user connect to qemu instance through ssh on localhost
   */

  class qemuBuilder
  {
    sb::socCnf topLevel;
    pid_t *clusters;

  public:
    qemuBuilder(std::ifstream & stream, char stype);
    ~qemuBuilder();
  };

}// end namespace qb
#endif /*_QEMU_BUILDER*/
