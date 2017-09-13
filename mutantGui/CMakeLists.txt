cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)
project(MUTANT_GUI)
# Include misceleanous scripts
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake-misc)

###
# Setup QT5 requirements
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets)

###
# Define external mutantGen path and machsuite_cls path and implied noc_helper path
set(MUTANTGEN_PATH ../apps-env/mutant)
set(MACHSUITECLS_PATH ../apps-env/machSuite-callStub)
set(NOCHELPER_PATH ../soc-env/models/noc/)
include_directories(${EXTERNAL_INCLUDE_DIRS} ${MUTANTGEN_PATH} ${MACHSUITECLS_PATH} ${NOCHELPER_PATH})

###
# List the project files
set(SOURCES
  mutantGui.cc
  popup.cc
  ${MUTANTGEN_PATH}/mutantGen.cc
  )

set(HEADERS
  mutantGui.h
  popup.h
  ${MUTANTGEN_PATH}/mutantGen.h
  ${MACHSUITECLS_PATH}/machSuite_cls_helper.h
  )

set(MOCS
    )

set(UIS
  mutantGui.ui
  popup.ui
  )

set(RESOURCES )

###
# Build and link the project
qt5_wrap_ui(GENERATED_SOURCES ${UIS})
qt5_add_resources(GENERATED_SOURCES ${RESOURCES})

# Build the app!
add_executable(mutant-gui
     MACOSX_BUNDLE
     WIN32
     # source files that are actually built direclty
     ${SOURCES}
     ${GENERATED_SOURCES}

     ${HEADERS}
     ${UIS}
     ${MOCS}
     ${RESOURCES}
     )

target_link_libraries(mutant-gui Qt5::Widgets)

###
# Generate Doxygen documentation
##############################
find_package(Doxygen)
if(DOXYGEN_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc ALL
     COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
     COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT} &> /dev/null
     COMMAND ${CMAKE_COMMAND} -E echo "Done."
     )
endif(DOXYGEN_FOUND)

## Generate clang_complete
# include(GenClangComplete)
# gen_clangComplete()
