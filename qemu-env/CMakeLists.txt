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
#Require qemu-xlnx
find_package(QemuXlnx)
if(QEMUXLNX_FOUND)
    set(QEMUXLNX_PATH ${QEMUXLNX_DIRS})
else()
    message( FATAL_ERROR "qemu-xlnx not found. use git submodule update --init --recursive to fetch qemu-xlnx source. Then don't forget to patch it (make patch_qemu)" )
endif()

# Require Cereal
find_package(Cereal)
if(CEREAL_FOUND)
    set(EXTERNAL_INCLUDE_DIRS ${EXTERNAL_INCLUDE_DIRS} ${CEREAL_INCLUDE_DIRS})
else()
    message( FATAL_ERROR "Cereal library not found." )
endif()

find_package(Petalinux)
if(PETALINUX_FOUND)
    set(PETALINUX_PATH ${PETALINUX_PATH})
else()
    message( WARNING "Petalinux not found. Some features will not be available, use prebuild binaries." )
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


###
# Manage qemu build
##############################
include(ExternalProject)
ExternalProject_Add(qemu_xlnx
    SOURCE_DIR ${QEMUXLNX_PATH}
    CONFIGURE_COMMAND ${QEMUXLNX_PATH}/configure --prefix=${CMAKE_CURRENT_BINARY_DIR} --target-list=aarch64-softmmu --enable-fdt --disable-kvm --disable-xen --python=/usr/bin/python2
    BUILD_COMMAND ${MAKE}
    )
### Extra target for patching libsystemctlm-soc submodules
add_custom_target( patch_qemu
  COMMAND cd ${QEMUXLNX_PATH} && git apply --check ${CMAKE_CURRENT_SOURCE_DIR}/submodules-patch/enable_emuHmpsoc_generic_devices.patch && git am --signoff <${CMAKE_CURRENT_SOURCE_DIR}/submodules-patch/enable_emuHmpsoc_generic_devices.patch
   )
# This patch command need to be applied manually

### Extra command to copy prebuild binaries at the good place
# Use with caution replace petalinux project if exist
add_custom_target( load_binaries
  COMMAND  mkdir -p  ${CMAKE_BINARY_DIR}/${PETALINUX_NAME}/images/linux &&  cp ${EMU_TOP_DIR}/binRelease/zImage ${CMAKE_BINARY_DIR}/${PETALINUX_NAME}/images/linux/zImage &&  cp ${EMU_TOP_DIR}/binRelease/rootfs.cpio.gz ${CMAKE_BINARY_DIR}/${PETALINUX_NAME}/images/linux/rootfs.cpio.gz &&  mkdir -p  ${CMAKE_BINARY_DIR}/qemu-env &&  cp ${EMU_TOP_DIR}/binRelease/emu-hmpsoc_t1_6.dtb ${CMAKE_BINARY_DIR}/emu-hmpsoc_t1_6.dtb &&  cp ${EMU_TOP_DIR}/binRelease/emu-hmpsoc_t7_12.dtb ${CMAKE_BINARY_DIR}/emu-hmpsoc_t7_12.dtb && mkdir -p ${EMU_TOP_BIN}/apps-env && cp ${EMU_TOP_DIR}/binRelease/sdImage_pkg.bin ${EMU_TOP_BIN}/apps-env/sdImage_pkg.bin
  )
#This command need to be used manually
set_target_properties(patch_qemu load_binaries PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

###
# Settings executable build
##############################
# Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} ${EMU_TOP_DIR}/soc-env)
## define executable
add_executable(vp_qemuLauncher $<TARGET_OBJECTS:QEMU_UTILITY> qemuLauncher.cc)
install(TARGETS vp_qemuLauncher DESTINATION bin)
add_dependencies( vp_qemuLauncher qemu_xlnx)

###
# Extra target for working on custom petalinux project
##############################
## Generate petalinux project
add_custom_target( init_ptx
    COMMAND ${EMU_TOP_DIR}/cmake-misc/initPtxProject.sh ${PETALINUX_PATH} ${CMAKE_CURRENT_BINARY_DIR} ${PETALINUX_NAME}
   )

 # Update device tree binaries
 add_custom_target( comp_dtc
  COMMAND ${PETALINUX_PATH}/tools/linux-i386/petalinux/bin/dtc -I dts -O dtb -o ${CMAKE_CURRENT_BINARY_DIR}/emu-hmpsoc_t1_6.dtb ${CMAKE_SOURCE_DIR}/device-tree/system-top_t1-6.dts
  COMMAND ${PETALINUX_PATH}/tools/linux-i386/petalinux/bin/dtc -I dts -O dtb -o ${CMAKE_CURRENT_BINARY_DIR}/emu-hmpsoc_t7_12.dtb ${CMAKE_SOURCE_DIR}/device-tree/system-top_t7-12.dts
  COMMAND ${PETALINUX_PATH}/tools/linux-i386/petalinux/bin/dtc -I dts -O dtb -o ${CMAKE_CURRENT_BINARY_DIR}/emu-hmpsoc_full.dtb ${CMAKE_SOURCE_DIR}/device-tree/system-top_full.dts
   )
set_target_properties(init_ptx comp_dtc PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

## Generate clang_complete
include(GenClangComplete)
gen_clangComplete()
