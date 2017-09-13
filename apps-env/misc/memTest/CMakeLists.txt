cmake_minimum_required(VERSION 2.8)
project(misc_memTest)

###
# Settings objects build
##############################
## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

## retrieve file list
file(GLOB SRC_MEMTEST "*.c")

## generate object library
add_library(MISC_MEMTEST OBJECT ${SRC_MEMTEST})

###
# Define executable target
##############################
add_executable(misc_memTest.${ARM_EXT} $<TARGET_OBJECTS:MISC_MEMTEST>)
target_link_libraries(misc_memTest.${ARM_EXT})
install(TARGETS misc_memTest.${ARM_EXT} DESTINATION bin)
