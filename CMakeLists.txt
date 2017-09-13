cmake_minimum_required(VERSION 2.8)
project(EMU_HMPSOC)

#version numbering
set (EMU_HMPSOC_VERSION_MAJOR 0)
set (EMU_HMPSOC_VERSION_MINOR 1)

###
# Define subpart of project as external cmake projects
##############################
include(ExternalProject)
### soc-env sub-project as vp_soc target
ExternalProject_Add( vp_soc
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/soc-env
  BINARY_DIR ${CMAKE_BINARY_DIR}/soc-env
  INSTALL_COMMAND ""
  CMAKE_ARGS -DEMU_TOP_DIR=${CMAKE_SOURCE_DIR} -DEMU_TOP_BIN=${CMAKE_BINARY_DIR} -DSYSTEMC_PREFIX=${SYSTEMC_PREFIX}
  )


set(PETALINUX_NAME "cosimPtx-build")
### qemu-env sub-project as  vp_launcher
ExternalProject_Add( vp_launcher
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/qemu-env
  BINARY_DIR ${CMAKE_BINARY_DIR}/qemu-env
  INSTALL_COMMAND ""
  CMAKE_ARGS -DEMU_TOP_DIR=${CMAKE_SOURCE_DIR} -DEMU_TOP_BIN=${CMAKE_BINARY_DIR} -DPETALINUX_PREFIX=${PETALINUX_PREFIX} -DPETALINUX_NAME=${PETALINUX_NAME}
  )

### apps-env sub-project as vp_apps
# Target disable in default list: required extra dependencies
# you could use the released binaries instead.
ExternalProject_Add( vp_apps
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/apps-env
  BINARY_DIR ${CMAKE_BINARY_DIR}/apps-env
  INSTALL_COMMAND ""
  CMAKE_ARGS -DEMU_TOP_DIR=${CMAKE_SOURCE_DIR} -DEMU_TOP_BIN=${CMAKE_BINARY_DIR} -DPETALINUX_PREFIX=${PETALINUX_PREFIX} -DPETALINUX_NAME=${PETALINUX_NAME}
  )
set_target_properties(vp_apps PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

### mutantGui sub-project as vp_mutantGui
ExternalProject_Add( vp_mutantGui
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/mutantGui
  BINARY_DIR ${CMAKE_BINARY_DIR}/mutantGui
  INSTALL_COMMAND ""
  CMAKE_ARGS -DEMU_TOP_DIR=${CMAKE_SOURCE_DIR} -DEMU_TOP_BIN=${CMAKE_BINARY_DIR}
  )
