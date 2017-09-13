cmake_minimum_required(VERSION 2.8)
find_package(Doxygen) #find host doxygen exec not the arm toolchain one
set(CMAKE_TOOLCHAIN_FILE ${EMU_TOP_DIR}/cmake-misc/toolchain_arm.cmake)
project(EMU_APPS)

#version numbering
set (EMU_APPS_VERSION_MAJOR 0)
set (EMU_APPS_VERSION_MINOR 1)

# Include misceleanous scripts and define toolchain
list(APPEND CMAKE_MODULE_PATH ${EMU_TOP_DIR}/cmake-misc)

###
# Required library
##############################
find_package(Petalinux)
if(PETALINUX_FOUND)
    set(PETALINUX_PATH ${PETALINUX_PATH})
else()
    message( WARNING "Petalinux not found. Drivers compilation and application packaging disable" )
endif()

# Require machSuite-kernel
find_package(machSuite)
if(MACHSUITE_FOUND)
    set(EXTERNAL_INCLUDE_DIRS ${EXTERNAL_INCLUDE_DIRS} ${MACHSUITE_INCLUDE_DIRS})
else()
    message( FATAL_ERROR "machSuite-kernel library not found." )
endif()

###
# Settings base include directories
##############################
set(EXTERNAL_INCLUDE_DIRS ${EXTERNAL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} ${EMU_TOP_DIR}/soc-env/models/noc/)

###
# Set Compilation Flags
##############################
set (DBG_LVL 0)
set (OPT_LVL fast)
### Define "name"_DEBUG to enable debug in module name
# Available debug flags are: -DNAPI_DEBUG -DLOCAL_CPU_BUFFER -DDATA_DUMP_DEBUG -DCLMGMT_DEBUG -DMACHSUITE_DEBUG -DMEXEC_DEBUG
set(CMAKE_C_FLAGS "-Wall -Wextra -Wpedantic -g${DBG_LVL} -O${OPT_LVL} -ldl")
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -Wpedantic -Wfatal-errors -g${DBG_LVL} -O${OPT_LVL} -ldl")
# set arm binary extension
set(ARM_EXT elf)

###
# Settings subdirectory build
##############################
add_subdirectory(api)
add_subdirectory(misc/memTest)
add_subdirectory(misc/mallocTest)
add_subdirectory(misc/nocTest)
add_subdirectory(machSuite-callStub)
add_subdirectory(${MACHSUITE_INCLUDE_DIRS} ${CMAKE_BINARY_DIR}/machsuite-kernel)
add_subdirectory(misc/mgmtTest)
add_subdirectory(misc/sampleTest)
add_subdirectory(misc/localComTest)
add_subdirectory(misc/comTask)
add_subdirectory(mutant)


###
# Custom cmd to call KBuild for modules drivers
##############################
add_custom_target( vp_drivers
  COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/drivers
  COMMAND cp -u ${CMAKE_CURRENT_SOURCE_DIR}/drivers/Kbuild ${CMAKE_CURRENT_BINARY_DIR}/drivers
  COMMAND make -C ${EMU_TOP_BIN}/qemu-env/${PETALINUX_NAME}/build/linux/kernel/xlnx-4.6 M=${CMAKE_CURRENT_BINARY_DIR}/drivers SRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}/drivers ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE_PATH}/bin/${CROSS_COMPILE_NAME}-
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/drivers
  SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/drivers)

add_custom_target( vp_appsPkg
  COMMAND ${EMU_TOP_DIR}/cmake-misc/updtApps.sh ${CMAKE_BINARY_DIR} ${EMU_TOP_DIR}
  DEPENDS vp_drivers misc_memTest.elf misc_mallocTest.elf misc_nocTest.elf misc_mgmtTest.elf misc_sampleTest.elf misc_localComTest.elf misc_comTask.elf mutant_exec.elf mutant_slave.elf
  SOURCES ${EMU_TOP_DIR}/cmake-misc/updtApps.sh)

# Exclude target from default and all to enable => these targets need extra dependencies on petalinux
set_target_properties(vp_drivers vp_appsPkg PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

###
# Generate Doxygen documentation
##############################
if(DOXYGEN_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc ALL
     COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
     COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT} &> /dev/null
     COMMAND ${CMAKE_COMMAND} -E echo "Done."
     )
endif(DOXYGEN_FOUND)


###
# Generate clang_complete
##############################
include(GenClangComplete)
gen_clangComplete()
