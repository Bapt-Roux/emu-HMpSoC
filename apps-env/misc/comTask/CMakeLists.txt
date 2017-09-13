cmake_minimum_required(VERSION 2.8)
project(misc_comTask)

###
# Settings objects build
##############################
## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/api ${CMAKE_SOURCE_DIR}/drivers ${CMAKE_SOURCE_DIR}/machSuite-callStub)

## retrieve file list
file(GLOB SRC_COMTASK "*.cc")

## generate object library
add_library(MISC_COMTASK OBJECT ${SRC_COMTASK})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

###
# Define executable target
##############################
add_executable(misc_comTask.${ARM_EXT} $<TARGET_OBJECTS:API_ALL> $<TARGET_OBJECTS:MACHSUITE_CLS> $<TARGET_OBJECTS:MACHSUITE_KER> $<TARGET_OBJECTS:MISC_COMTASK>)
target_link_libraries(misc_comTask.${ARM_EXT})
install(TARGETS misc_comTask.${ARM_EXT} DESTINATION bin)
