# - GenClangComplete
# This module generate clang complete configuration file for project
#
# Example Usages:
# gen_clangComplete()
#=============================================================================
function(gen_clangComplete )
message(STATUS "Generarating ${CMAKE_SOURCE_DIR}/.clang_complete")
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
file(WRITE ${CMAKE_SOURCE_DIR}/.clang_complete "")

foreach(dir ${dirs})
  file(APPEND ${CMAKE_SOURCE_DIR}/.clang_complete "-I${dir}\n")
endforeach()

string(REPLACE "'" "" CMAKE_CXX_FLAGS_SPLIT ${CMAKE_CXX_FLAGS})
string(REPLACE " " ";" CMAKE_CXX_FLAGS_SPLIT ${CMAKE_CXX_FLAGS_SPLIT})
foreach(flag ${CMAKE_CXX_FLAGS_SPLIT})
  file(APPEND ${CMAKE_SOURCE_DIR}/.clang_complete "${flag}\n")
endforeach()
# Disable warning unused parameter
file(APPEND ${CMAKE_SOURCE_DIR}/.clang_complete "-Wno-unused-parameter\n")
endfunction(gen_clangComplete)

