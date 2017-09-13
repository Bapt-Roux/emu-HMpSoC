cmake_minimum_required(VERSION 2.8)
project(apps_api)

###
# Settings objects build
##############################
## Include directories
include_directories(${EXTERNAL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/drivers)

## nocAPI object
file(GLOB SRC_NOCAPI "nocAPI.cc")
add_library(API_NOC OBJECT ${SRC_NOCAPI})

## Custom malloc library
file(GLOB SRC_ALLOCAPI "monAlloc.c")
add_library(API_ALLOC  OBJECT ${SRC_ALLOCAPI})

## clMgmtAPI object
file(GLOB SRC_CLMGMTAPI "clMgmtAPI.cc")
add_library(API_CLMGMT OBJECT ${SRC_CLMGMTAPI})
add_library(API_ALL OBJECT ${SRC_CLMGMTAPI} ${SRC_ALLOCAPI} ${SRC_NOCAPI})
