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
