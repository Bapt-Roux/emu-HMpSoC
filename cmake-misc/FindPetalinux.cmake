# - Find Petalinux
# This module determines where the cereal include files.
#
# For these components the following variables are set:
#
#  PETALINUX_FOUND                    - TRUE if all components are found.
#  PETALINUX_PATH                     - Full path.
#
# Example Usages:
#  FIND_PACKAGE(Petalinux)
#
#=============================================================================

message(STATUS "Searching for Petalinux")

# The HINTS option should only be used for values computed from the system.
SET(_PETALINUX_HINTS
  $ENV{PETALINUX_HOME}
  ${PETALINUX_PREFIX}
  )

FIND_PATH(PETALINUX_PATH
  NAMES settings.sh
  HINTS ${_PETALINUX_HINTS}
  )

if(NOT("${PETALINUX_PATH}" MATCHES "PETALINUX_PATH-NOTFOUND"))
    set(PETALINUX_FOUND TRUE)
endif(NOT("${PETALINUX_PATH}" MATCHES "PETALINUX_PATH-NOTFOUND"))

message(STATUS "Petalinux path = ${PETALINUX_PATH}")
