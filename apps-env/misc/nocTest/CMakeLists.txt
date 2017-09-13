cmake_minimum_required(VERSION 2.8)
project(misc_nocTest)

###
# Settings objects build
##############################
## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

## retrieve file list
file(GLOB SRC_NOCTEST "*.cc")

## generate object library
add_library(MISC_NOCTEST OBJECT ${SRC_NOCTEST})

###
# Define executable target
##############################
add_executable(misc_nocTest.${ARM_EXT} $<TARGET_OBJECTS:MISC_NOCTEST>)
target_link_libraries(misc_nocTest.${ARM_EXT})
install(TARGETS misc_nocTest.${ARM_EXT} DESTINATION bin)
