cmake_minimum_required(VERSION 2.8)
project(QEMU_PATCH)

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
    message( FATAL_ERROR "qemu-xlnx not found. use git submodule update --init --recursive to fetch qemu-xlnx source" )
endif()

###
# Extra target
##############################
## Generate petalinux project
add_custom_target( patch_qemu
  COMMAND cd ${QEMUXLNX_PATH} && git am --signoff <${CMAKE_CURRENT_SOURCE_DIR}/enable_emuHmpsoc_generic_devices.patch
   )

include(ExternalProject)
ExternalProject_Add(qemu_xlnx
    SOURCE_DIR ${QEMUXLNX_PATH}
    CONFIGURE_COMMAND ${QEMUXLNX_PATH}/configure --prefix=${CMAKE_CURRENT_BINARY_DIR} --target-list=aarch64-softmmu --enable-fdt --disable-kvm --disable-xen --python=/usr/bin/python2
    BUILD_COMMAND ${MAKE}
    DEPENDS patch_qemu
    )
