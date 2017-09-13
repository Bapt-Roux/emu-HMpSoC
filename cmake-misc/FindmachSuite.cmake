# - Find machSuite-kernel
# This module determines where the cereal include files.
#
# For these components the following variables are set:
#
#  MACHSUITE_FOUND                    - TRUE if all components are found.
#  MACHSUITE_INCLUDE_DIRS             - Full paths to all includes dirs.
#
# Example Usages:
#  FIND_PACKAGE(machSuite)
#
#=============================================================================

message(STATUS "Searching for machsuite-kernel")

# The HINTS option should only be used for values computed from the system.
SET(_MACHSUITE_HINTS
  ${CMAKE_SOURCE_DIR}/../machSuite-kernel
  )

FIND_PATH(MACHSUITE_INCLUDE_DIRS
  NAMES aes.h
  HINTS ${_MACHSUITE_HINTS}
  )

if(NOT("${MACHSUITE_INCLUDE_DIRS}" MATCHES "MACHSUITE_INCLUDE_DIRS-NOTFOUND"))
    set(MACHSUITE_FOUND TRUE)
endif(NOT("${MACHSUITE_INCLUDE_DIRS}" MATCHES "MACHSUITE_INCLUDE_DIRS-NOTFOUND"))

message(STATUS "machSuite-kernel include = ${MACHSUITE_INCLUDE_DIRS}")
