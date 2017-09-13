cmake_minimum_required(VERSION 2.8)
project(misc_sampleTest)

###
# Settings objects build
##############################
## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/machSuite-callStub)

## retrieve file list
file(GLOB SRC_SAMPLETEST "*.cc")

## generate object library
add_library(MISC_SAMPLETEST OBJECT ${SRC_SAMPLETEST})

###
# Define executable target
##############################
add_executable(misc_sampleTest.${ARM_EXT} $<TARGET_OBJECTS:API_ALLOC> $<TARGET_OBJECTS:MACHSUITE_CLS> $<TARGET_OBJECTS:MACHSUITE_KER> $<TARGET_OBJECTS:MISC_SAMPLETEST>)
target_link_libraries(misc_sampleTest.${ARM_EXT})
install(TARGETS misc_sampleTest.${ARM_EXT} DESTINATION bin)
