cmake_minimum_required(VERSION 2.8)
project(mutantExec)

###
# Settings objects build
##############################
## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/api ${CMAKE_SOURCE_DIR}/drivers ${CMAKE_SOURCE_DIR}/machSuite-callStub)

## retrieve file list
file(GLOB SRC_MUTANTEXEC "mutantExec.cc" "mutantGen.cc")
file(GLOB SRC_MUTANTSLAVE "mutantSlave.cc")

## generate object library
add_library(MUTANTEXEC OBJECT ${SRC_MUTANTEXEC})
add_library(MUTANTSLAVE OBJECT ${SRC_MUTANTSLAVE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

###
# Define executable target
##############################
add_executable(mutant_exec.${ARM_EXT} $<TARGET_OBJECTS:API_ALL> $<TARGET_OBJECTS:MACHSUITE_CLS> $<TARGET_OBJECTS:MACHSUITE_KER> $<TARGET_OBJECTS:MUTANTEXEC>)
target_link_libraries(mutant_exec.${ARM_EXT})
install(TARGETS mutant_exec.${ARM_EXT} DESTINATION bin)

add_executable(mutant_slave.${ARM_EXT} $<TARGET_OBJECTS:API_ALL> $<TARGET_OBJECTS:MACHSUITE_CLS> $<TARGET_OBJECTS:MACHSUITE_KER> $<TARGET_OBJECTS:MUTANTSLAVE>)
target_link_libraries(mutant_slave.${ARM_EXT})
install(TARGETS mutant_slave.${ARM_EXT} DESTINATION bin)
