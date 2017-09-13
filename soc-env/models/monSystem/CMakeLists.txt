cmake_minimum_required(VERSION 2.8)
project(models_monSystem)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

###
# Required library
##############################

# Require SYSTEMC > 2.3
# TLM is include within systemC
#set(SystemC_USE_STATIC_LIBS ${USE_STATIC_SYSTEMC})
find_package(SystemC 2.3.1)
if(SystemC_FOUND)
    set(EXTERNAL_INCLUDE_DIRS ${EXTERNAL_INCLUDE_DIRS} ${SystemC_INCLUDE_DIRS})
else()
    message( FATAL_ERROR "SystemC library not found." )
endif()

# Require systemcTLMSoC
find_package(SCTlmSoC)
if(SCTLMSOC_FOUND)
    set(EXTERNAL_INCLUDE_DIRS ${EXTERNAL_INCLUDE_DIRS} ${SCTLMSOC_LIB_DIRS})
else()
    message( FATAL_ERROR "systemCTlmSoC library not found." )
endif()

# Require Cereal
find_package(Cereal)
if(CEREAL_FOUND)
    set(EXTERNAL_INCLUDE_DIRS ${EXTERNAL_INCLUDE_DIRS} ${CEREAL_INCLUDE_DIRS})
else()
    message( FATAL_ERROR "Cereal library not found." )
endif()

###
# Set Compilation Flags
##############################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSC_INCLUDE_DYNAMIC_PROCESSES")

###
# Settings objects build
##############################

## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR})

## retrieve file list
file(GLOB SRC_MONSYSTEM "*.cc")

## generate object library
add_library(MODELS_MONSYSTEM OBJECT ${SRC_MONSYSTEM})
