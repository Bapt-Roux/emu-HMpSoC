cmake_minimum_required(VERSION 2.8)
project(EMU_SW)

#version numbering
set (EMU_SW_VERSION_MAJOR 0)
set (EMU_SW_VERSION_MINOR 1)

# Include misceleanous scripts
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake-misc)

###
# Required library
##############################
#Require petalinux
find_package(Petalinux)
if(PETALINUX_FOUND)
    set(PETALINUX_PATH ${PETALINUX_PATH})
else()
    message( FATAL_ERROR "Petalinux not found." )
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
set (DBG_LVL 0)
set (OPT_LVL fast)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -Wpedantic -Wfatal-errors -g${DBG_LVL} -O${OPT_LVL}")

###
# Settings subdirectory build
##############################
add_subdirectory(utility)
add_subdirectory(qemu-patch)

###
# Settings executable build
##############################
# Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} ${EMU_TOP_DIR}/soc-env)
## define executable
add_executable(vp_qemuLauncher $<TARGET_OBJECTS:QEMU_UTILITY> qemuLauncher.cc)
# target_link_libraries(vp_qemuLauncher)
install(TARGETS vp_qemuLauncher DESTINATION bin)
# add_dependencies(vp_qemuLauncher init_dtc) //FIXME ATM generate linkage error


###
# Extra target
##############################
## Generate petalinux project
add_custom_target( init_ptx
    COMMAND ${EMU_TOP_DIR}/cmake-misc/initPtxProject.sh ${PETALINUX_PATH} ${CMAKE_CURRENT_BINARY_DIR} ${PETALINUX_NAME}
   )

 add_custom_target( comp_dtc
  COMMAND ${PETALINUX_PATH}/tools/linux-i386/petalinux/bin/dtc -I dts -O dtb -o ${CMAKE_CURRENT_BINARY_DIR}/emu-hmpsoc_t1_6.dtb ${CMAKE_SOURCE_DIR}/device-tree/system-top_t1-6.dts
  COMMAND ${PETALINUX_PATH}/tools/linux-i386/petalinux/bin/dtc -I dts -O dtb -o ${CMAKE_CURRENT_BINARY_DIR}/emu-hmpsoc_t7_12.dtb ${CMAKE_SOURCE_DIR}/device-tree/system-top_t7-12.dts
  COMMAND ${PETALINUX_PATH}/tools/linux-i386/petalinux/bin/dtc -I dts -O dtb -o ${CMAKE_CURRENT_BINARY_DIR}/emu-hmpsoc_full.dtb ${CMAKE_SOURCE_DIR}/device-tree/system-top_full.dts
   )

## Generate clang_complete
include(GenClangComplete)
gen_clangComplete()
