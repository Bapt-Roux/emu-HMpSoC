cmake_minimum_required(VERSION 2.8)
project(EMU_SOC)

#version numbering
set (EMU_SOC_VERSION_MAJOR 0)
set (EMU_SOC_VERSION_MINOR 1)

# Include misceleanous scripts
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake-misc)

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

# Require machSuite-kernel
find_package(machSuite)
if(MACHSUITE_FOUND)
    set(EXTERNAL_INCLUDE_DIRS ${EXTERNAL_INCLUDE_DIRS} ${MACHSUITE_INCLUDE_DIRS})
else()
    message( FATAL_ERROR "machSuite-kernel library not found." )
endif()

###
# Set Compilation Flags
##############################
set (DBG_LVL 0)
set (OPT_LVL fast)
### Define "name"_DEBUG to enable debug in module name
# Available debug flags are: -DMEMORY_DEBUG -DICONNECT_DEBUG -DMS_DEBUG -DNOC_DEBUG -DMONO_DEBUG -DNIPC_DEBUG -DHWIP_DEBUG -DMACHSUITE_DEBUG -DSB_DEBUG
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -Wpedantic -Wfatal-errors -g${DBG_LVL} -O${OPT_LVL}")

###
# Settings subdirectory build
##############################
add_subdirectory(models/monSystem)
add_subdirectory(models/structural)
add_subdirectory(models/noc)
add_subdirectory(models/fpga)
add_subdirectory(models/utility)
add_subdirectory(${MACHSUITE_INCLUDE_DIRS} ${CMAKE_BINARY_DIR}/machsuite-kernel)

###
# Settings systemcTlmSoC object build
##############################
file(GLOB SRC_SCTLMSOC_ZYNQ "${SCTLMSOC_LIB_DIRS}/zynq/*.cc")
file(GLOB SRC_SCTLMSOC_RP_CC "${SCTLMSOC_LIB_DIRS}/libremote-port/*.cc")
file(GLOB SRC_SCTLMSOC_RP_C "${SCTLMSOC_LIB_DIRS}/libremote-port/*.c")
message(STATUS "TLMSOC Zynq ${SRC_SCTLMSOC_ZYNQ}")
message(STATUS "TLMSOC remotePort [c++] ${SRC_SCTLMSOC_RP_CC}")
message(STATUS "TLMSOC remotePort [c] ${SRC_SCTLMSOC_RP_C}")

include_directories(${EXTERNAL_INCLUDE_DIRS} ${SCTLMSOC_INCLUDE_DIRS})
add_library(LIB_SCTLMSOC OBJECT ${SRC_SCTLMSOC_ZYNQ} ${SRC_SCTLMSOC_RP_CC} ${SRC_SCTLMSOC_RP_C})

###
# Settings executable build
##############################
## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} ${SCTLMSOC_INCLUDE_DIRS})

## define executable
add_executable(vp_mainFm $<TARGET_OBJECTS:LIB_SCTLMSOC> $<TARGET_OBJECTS:MODELS_MONSYSTEM> $<TARGET_OBJECTS:MODELS_STRUCT> $<TARGET_OBJECTS:MODELS_NOC> $<TARGET_OBJECTS:MODELS_FPGA> $<TARGET_OBJECTS:MACHSUITE_KER> $<TARGET_OBJECTS:MACHSUITE_HWS> $<TARGET_OBJECTS:MODELS_UTILITY> vp_mainFm.cc)
target_link_libraries(vp_mainFm ${SystemC_LIBRARIES} pthread dl rt)
install(TARGETS vp_mainFm DESTINATION bin)

###
# Generate Doxygen documentation
##############################
find_package(Doxygen)
if(DOXYGEN_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc ALL
     COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
     COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT} &> /dev/null
     COMMAND ${CMAKE_COMMAND} -E echo "Done."
     )
endif(DOXYGEN_FOUND)

## Generate clang_complete
include(GenClangComplete)
gen_clangComplete()
