cmake_minimum_required(VERSION 2.8)
project(misc_mallocTest)

###
# Settings objects build
##############################
## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/api ${CMAKE_SOURCE_DIR}/drivers )

## retrieve file list
file(GLOB SRC_MALLOCTEST "*.cc")

## generate object library
add_library(MISC_MALLOCTEST OBJECT ${SRC_MALLOCTEST})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

###
# Define executable target
##############################
add_executable(misc_mallocTest.${ARM_EXT} $<TARGET_OBJECTS:API_ALL> $<TARGET_OBJECTS:MACHSUITE_CLS> $<TARGET_OBJECTS:MACHSUITE_KER> $<TARGET_OBJECTS:MISC_MALLOCTEST>)
target_link_libraries(misc_mallocTest.${ARM_EXT})
install(TARGETS misc_mallocTest.${ARM_EXT} DESTINATION bin)
