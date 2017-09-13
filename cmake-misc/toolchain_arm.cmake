###
# This file define CMake cross compiling basic feature
# for ARM target
# User need to define PETALINUX_PREFIX
############################################################

# Toolchain name and version
SET(CMAKE_SYSTEM_NAME arm-linux)
SET(CMAKE_SYSTEM_VERSION 1)

# Cross compiler and cross environnement
SET(ARCH arm)
SET(CROSS_COMPILE_PATH ${PETALINUX_PREFIX}/tools/linux-i386/gcc-arm-linux-gnueabi)
SET(CROSS_COMPILE_NAME arm-linux-gnueabihf)
SET(CMAKE_C_COMPILER ${CROSS_COMPILE_PATH}/bin/${CROSS_COMPILE_NAME}-gcc)
SET(CMAKE_CXX_COMPILER ${CROSS_COMPILE_PATH}/bin/${CROSS_COMPILE_NAME}-g++)
SET(CMAKE_FIND_ROOT_PATH ${CROSS_COMPILE_PATH}/${CROSS_COMPILE_NAME})

# Search functionnality
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER)
