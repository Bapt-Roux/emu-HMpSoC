# - Find Cereal
# This module determines where the cereal include files.
#
# For these components the following variables are set:
#
#  CEREAL_FOUND                    - TRUE if all components are found.
#  CEREAL_INCLUDE_DIRS             - Full paths to all include dirs.
#
# Example Usages:
#  FIND_PACKAGE(Cereal)
#
#=============================================================================

message(STATUS "Searching for Cereal")

# The HINTS option should only be used for values computed from the system.
SET(_CEREAL_HINTS
  $ENV{CEREAL_HOME}
  ${CEREAL_PREFIX}
  ${CMAKE_SOURCE_DIR}/libcereal
  ${CMAKE_SOURCE_DIR}/../soc-env/libcereal
  )
# Hard-coded guesses should still go in PATHS. This ensures that the user
# environment can always override hard guesses.
SET(_CEREAL_PATHS
  /opt/cereal
  /usr/include/cereal
  /usr/lib
  )

FIND_PATH(CEREAL_INCLUDE_DIRS
  NAMES cereal
  HINTS ${_CEREAL_HINTS}
  PATHS ${_CEREAL_PATHS}
  PATH_SUFFIXES include
  )

if(NOT("${CEREAL_INCLUDE_DIRS}" MATCHES "CEREAL_INCLUDE_DIRS-NOTFOUND"))
    set(CEREAL_FOUND TRUE)
endif(NOT("${CEREAL_INCLUDE_DIRS}" MATCHES "CEREAL_INCLUDE_DIRS-NOTFOUND"))

message(STATUS "Cereal include = ${CEREAL_INCLUDE_DIRS}")
