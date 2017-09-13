cmake_minimum_required(VERSION 2.8)
project(hws_machsuite)

###
# Settings objects build
##############################
## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-label")

file(GLOB SRC_MACHSUITE_HWS "*.cc")
add_library(MACHSUITE_HWS OBJECT ${SRC_MACHSUITE_HWS})
