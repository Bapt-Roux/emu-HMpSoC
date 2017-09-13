# - Find systemCTlmSoC
# This module determines where the cereal include files.
#
# For these components the following variables are set:
#
#  SCTLMSOC_FOUND                    - TRUE if all components are found.
#  SCTLMSOC_LIB_DIRS             - Full paths to all submodules dirs.
#  SCTLMSOC_INCLUDE_DIRS             - Full paths to all includes dirs.
#
# Example Usages:
#  FIND_PACKAGE(SCTlmSoC)
#
#=============================================================================

message(STATUS "Searching for systemCTlmSoC")

# The HINTS option should only be used for values computed from the system.
SET(_SCTLMSOC_HINTS
  $ENV{SCTLMSOC_HOME}
  ${SCTLMSOC_PREFIX}
  ${CMAKE_SOURCE_DIR}/libsystemctlm-soc
  )

FIND_PATH(SCTLMSOC_LIB_DIRS
  NAMES zynq
  HINTS ${_SCTLMSOC_HINTS}
  )

set(SCTLMSOC_INCLUDE_DIRS ${SCTLMSOC_LIB_DIRS}/zynq ${SCTLMSOC_LIB_DIRS}/libremote-port ${SCTLMSOC_LIB_DIRS}/tlm-extensions)

if(NOT("${SCTLMSOC_LIB_DIRS}" MATCHES "SCTLMSOC_LIB_DIRS-NOTFOUND"))
    set(SCTLMSOC_FOUND TRUE)
endif(NOT("${SCTLMSOC_LIB_DIRS}" MATCHES "SCTLMSOC_LIB_DIRS-NOTFOUND"))

message(STATUS "systemCTlmSoC topDir = ${SCTLMSOC_LIB_DIRS}")
message(STATUS "systemCTlmSoC include = ${SCTLMSOC_INCLUDE_DIRS}")
