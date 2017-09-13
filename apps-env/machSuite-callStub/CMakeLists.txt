cmake_minimum_required(VERSION 2.8)
project(cls_machsuite)

###
# Settings objects build
##############################
## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-label")

file(GLOB SRC_MACHSUITE_CLS "*.cc")
add_library(MACHSUITE_CLS OBJECT ${SRC_MACHSUITE_CLS})
