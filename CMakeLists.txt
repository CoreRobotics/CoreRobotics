# Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
# Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
# http://www.corerobotics.org

# ------------------------------------------------------------------------------
# Settings:
#   -Dall    = [true/false]         all targets
#   -Dfull   = [true/false]         include deprecated source
#   -Dlib    = [true/false]         c++ lib
#   -Dtests  = [true/false]         c++ tests
#   -Dreport = [true/false]         sets all options for lcov report
#   -Dlint   = [true/false]         runs clang-tiny linting on source code
#   -Dformat = [true/false]         runs clang-format -i on source code
#   -Dpython = [true,false,2,3]     python binding
# ------------------------------------------------------------------------------

# cmake settings
cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_policy(SET CMP0054 NEW)

# Set the project version
set(CR_MAJOR_VERSION 2)
set(CR_MINOR_VERSION 0)
set(CR_PATCH_VERSION 0)
set(CR_VERSION ${CR_MAJOR_VERSION}.${CR_MINOR_VERSION}.${CR_PATCH_VERSION})

# Define the project name and other CoreRobotics parameters
set(PROJECT_NAME CoreRobotics)
project(${PROJECT_NAME} VERSION ${CR_VERSION})

# Turn on useful compiler options
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# ------------------------------------------------------------------------------
# Find depdendencies
# ------------------------------------------------------------------------------
find_package (Eigen3 3.3.3 REQUIRED)
find_package (Boost 1.58.0 REQUIRED)
find_package (GTest)
find_program (CLANG_TIDY_EXE NAMES "clang-tidy")
find_program (CLANG_FORMAT_EXE NAMES "clang-format")

# ------------------------------------------------------------------------------
# Global library variables (visible to subdirectory CMakeLists)
# ------------------------------------------------------------------------------
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# define includes
set(CR_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

# set include search paths
include_directories(${CR_INCLUDE_DIRS})

# define source directories
set(CR_SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src)

# define modules, target prefix, and module dependencies
set(CR_LIB_PREFIX "cr-")
set(CR_LIB_MODULES core runtime math physics signal noise model world control estimation)
set(CR_LIB_DEP_core)
set(CR_LIB_DEP_runtime ${CR_LIB_PREFIX}core)
set(CR_LIB_DEP_math)
set(CR_LIB_DEP_physics)
set(CR_LIB_DEP_signal ${CR_LIB_PREFIX}core)
set(CR_LIB_DEP_noise ${CR_LIB_PREFIX}math)
set(CR_LIB_DEP_model ${CR_LIB_PREFIX}core ${CR_LIB_PREFIX}math ${CR_LIB_PREFIX}noise)
set(CR_LIB_DEP_world ${CR_LIB_PREFIX}core ${CR_LIB_PREFIX}physics)
set(CR_LIB_DEP_control ${CR_LIB_PREFIX}core)
set(CR_LIB_DEP_estimation ${CR_LIB_PREFIX}model ${CR_LIB_PREFIX}noise)

# get header and source files
set(CR_HEADERS "")
set(CR_SOURCES "")
set(CR2_FILES "")
foreach(p ${CR_LIB_MODULES})
  file(GLOB CR_HEADERS_ ${CMAKE_CURRENT_SOURCE_DIR}/src/${p}/CR*.hpp)
  file(GLOB CR_SOURCES_ ${CMAKE_CURRENT_SOURCE_DIR}/src/${p}/CR*.cpp)
  file(GLOB CR_HEADERS_${p} ${CMAKE_CURRENT_SOURCE_DIR}/src/${p}/*.hpp)
  file(GLOB CR_SOURCES_${p} ${CMAKE_CURRENT_SOURCE_DIR}/src/${p}/*.cpp)
  list(APPEND CR2_FILES ${CR_HEADERS_${p}} ${CR_SOURCES_${p}})
  foreach(f ${CR_HEADERS_})
    list(REMOVE_ITEM CR_HEADERS_${p} "${f}")
  endforeach()
  foreach(f ${CR_SOURCES_})
    list(REMOVE_ITEM CR_SOURCES_${p} "${f}")
  endforeach()
  list(APPEND CR_HEADERS ${CR_HEADERS_})
  list(APPEND CR_SOURCES ${CR_SOURCES_})
endforeach()

# get the tests
file(GLOB CR2_TESTS
  ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp
)

# If building on linux add pthread and rt
if(UNIX AND NOT APPLE)
    set(CR_LINK_FLAGS pthread rt)
else()
    set(CR_LINK_FLAGS "")
endif()

# set CR_LIB_TARGETS
set(CR_LIB_TARGETS "")
if (full)
  set(CR_LIB_TARGETS CoreRobotics)
endif()
foreach(p ${CR_LIB_MODULES})
  list(APPEND CR_LIB_TARGETS ${CR_LIB_PREFIX}${p})
endforeach()

# ------------------------------------------------------------------------------
# Code coverage with lcov
# ------------------------------------------------------------------------------
add_compile_options(-Wall -pedantic)
add_library(coverage_config INTERFACE)
 
if (report)
  # Add required flags (GCC & LLVM/Clang)
  target_compile_options(coverage_config INTERFACE
    -O0        # no optimization
    -g         # generate debug info
    --coverage # sets all required flags
  )
  target_link_libraries(coverage_config INTERFACE --coverage)
endif()

enable_testing() # CTest

# ------------------------------------------------------------------------------
# C++ Formatting
# ------------------------------------------------------------------------------
if (format)
  if(NOT CLANG_FORMAT_EXE)
    message(STATUS "clang-format not found, skipping format")
    SET(format FALSE)
  else()
    message(STATUS "clang-format found: ${CLANG_FORMAT_EXE}")
    add_custom_target(
      run-format
      COMMAND ${CLANG_FORMAT_EXE} -i -style=file ${CR2_FILES} ${CR2_TESTS}
    )
  endif()
endif()

# ------------------------------------------------------------------------------
# C++ Linting
# ------------------------------------------------------------------------------
if (lint)
  if(NOT CLANG_TIDY_EXE)
    message(STATUS "clang-tidy not found.")
    SET(lint FALSE)
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    set(DO_CLANG_TIDY "${CLANG_TIDY_EXE}" "-checks=*")
  endif()
endif()

# ------------------------------------------------------------------------------
# Setup the library targets
# ------------------------------------------------------------------------------
add_subdirectory(${PROJECT_SOURCE_DIR}/src)

# ------------------------------------------------------------------------------
# Setup the test target
# ------------------------------------------------------------------------------
if ((tests OR all OR report) AND (GTest_FOUND))
  add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
endif()

# ------------------------------------------------------------------------------
# Setup the python target
# ------------------------------------------------------------------------------
if(python OR all)
  add_subdirectory(${PROJECT_SOURCE_DIR}/python)
endif()

# ------------------------------------------------------------------------------
# Export the package configuration and version files
# ------------------------------------------------------------------------------
if (NOT report)

  # export package for use from the build tree
  export(
    TARGETS ${CR_LIB_TARGETS}
    FILE ${PROJECT_BINARY_DIR}/CoreRoboticsTargets.cmake
  )
  export (PACKAGE CoreRobotics)

  # package definitions
  set (CONF_INCLUDE_DIRS ${CR_INCLUDE_DIRS})
  set (CONF_LIBRARIES    ${CR_LIB_TARGETS})
  set (CONF_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/lib)
  set (CONF_SOURCE_DIR   ${PROJECT_SOURCE_DIR})

  # package configuration and version files
  configure_file(
    CoreRoboticsConfig.cmake.in
    "${PROJECT_BINARY_DIR}/CoreRoboticsConfig.cmake" @ONLY
  )
  configure_file(
    CoreRoboticsConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/CoreRoboticsConfigVersion.cmake" @ONLY
  )
endif()
