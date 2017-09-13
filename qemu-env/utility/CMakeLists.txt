cmake_minimum_required(VERSION 2.8)
project(qemu_utility)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

###
# Set Compilation Flags
##############################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPETALINUX_PATH=${PETALINUX_PATH} -DEMU_TOP_BIN=${EMU_TOP_BIN} -DEMU_CUR_BIN=${CMAKE_BINARY_DIR} -DPETALINUX_NAME=${PETALINUX_NAME}")

###
# Settings objects build
##############################
## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} ${CEREAL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/../soc-env/)
## retrieve file list
file(GLOB SRC_UTILITY "*.cc")
## generate object library
add_library(QEMU_UTILITY OBJECT ${SRC_UTILITY})
