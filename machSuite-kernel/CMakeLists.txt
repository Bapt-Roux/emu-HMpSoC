cmake_minimum_required(VERSION 2.8)
project(machSuite_ker)

###
# Settings objects build
##############################
## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-label")

file(GLOB SRC_MACHSUITE_KER "*.cc")
set_source_files_properties(backprop.cc PROPERTIES COMPILE_FLAGS -O2)

add_library(MACHSUITE_KER OBJECT ${SRC_MACHSUITE_KER})
