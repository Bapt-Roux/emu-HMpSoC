# - Find qemuXlnx
# This module determines where the cereal include files.
#
# For these components the following variables are set:
#
#  QEMUXLNX_FOUND                    - TRUE if all components are found.
#  QEMUXLNX_DIRS                     - Full paths to all submodules dirs.
#
# Example Usages:
#  FIND_PACKAGE(QemuXlnx)
#
#=============================================================================

message(STATUS "Searching for qemuXlnx")

# The HINTS option should only be used for values computed from the system.
SET(_QEMUXLNX_HINTS
  $ENV{QEMUXLNX_HOME}
  ${QEMUXLNX_PREFIX}
  ${CMAKE_SOURCE_DIR}/qemu-xlnx
  )

FIND_PATH(QEMUXLNX_DIRS
  NAMES include/qemu
  HINTS ${_QEMUXLNX_HINTS}
  )

if(NOT("${QEMUXLNX_DIRS}" MATCHES "QEMUXLNX_DIRS-NOTFOUND"))
    set(QEMUXLNX_FOUND TRUE)
endif(NOT("${QEMUXLNX_DIRS}" MATCHES "QEMUXLNX_DIRS-NOTFOUND"))

message(STATUS "qemuXlnx topDir = ${QEMUXLNX_DIRS}")
