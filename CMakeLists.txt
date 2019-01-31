# =====================================================================
#
# Software License Agreement (BSD-3-Clause License)
# Copyright (c) 2019, CoreRobotics.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# * Neither the name of CoreRobotics nor the names of its contributors
# may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# \project CoreRobotics Project
# \url     www.corerobotics.org
#
#
# =====================================================================

# ---------------------------------------------------------------------
# Settings:
#   -Dall = [true,false]            all targets
#   -Dfull = [true,false]           include deprecated source
#   -Dlib = [true,false]            c++ lib
#   -Dtests = [true,false]          c++ tests
#   -Dreport = [true,false]         sets all options for lcov report
#   -Dpython = [true,false,2,3]     python binding
# ---------------------------------------------------------------------

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



# ---------------------------------------------------------------------
# Find depdendencies
# ---------------------------------------------------------------------

# If building wrappers Eigen 3.3.3 is required
set (EIGEN_VERSION 3.3.3)
find_package (Eigen3 ${EIGEN_VERSION})
find_package (Boost 1.48)
find_package (GTest)
find_package (OpenSceneGraph)
find_package (SWIG)

if(NOT Boost_INCLUDE_DIR)
  message(FATAL_ERROR "Boost_INCLUDE_DIR not found - please define the path to boost")
endif()

if(NOT EIGEN3_INCLUDE_DIR)
  message(FATAL_ERROR "EIGEN3_INCLUDE_DIR not found - please define the path to eigen")
else()
  message(STATUS "Eigen found at ${EIGEN3_INCLUDE_DIR}")
endif()


# ---------------------------------------------------------------------
# Global library variables (visible to subdirectory CMakeLists)
# ---------------------------------------------------------------------

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
set(CR_LIB_MODULES core math physics signal noise model world control)
set(CR_LIB_DEP_core)
set(CR_LIB_DEP_math)
set(CR_LIB_DEP_physics)
set(CR_LIB_DEP_signal ${CR_LIB_PREFIX}core)
set(CR_LIB_DEP_noise ${CR_LIB_PREFIX}math)
set(CR_LIB_DEP_model ${CR_LIB_PREFIX}core ${CR_LIB_PREFIX}math)
set(CR_LIB_DEP_world ${CR_LIB_PREFIX}core ${CR_LIB_PREFIX}physics)
set(CR_LIB_DEP_control ${CR_LIB_PREFIX}core)

# get header and source files
set(CR_HEADERS "")
set(CR_SOURCES "")
foreach(p ${CR_LIB_MODULES})
  file(GLOB CR_HEADERS_ ${CMAKE_CURRENT_SOURCE_DIR}/src/${p}/CR*.hpp)
  file(GLOB CR_SOURCES_ ${CMAKE_CURRENT_SOURCE_DIR}/src/${p}/CR*.cpp)
  file(GLOB CR_HEADERS_${p} ${CMAKE_CURRENT_SOURCE_DIR}/src/${p}/*.hpp)
  file(GLOB CR_SOURCES_${p} ${CMAKE_CURRENT_SOURCE_DIR}/src/${p}/*.cpp)
  foreach(f ${CR_HEADERS_})
    list(REMOVE_ITEM CR_HEADERS_${p} "${f}")
  endforeach()
  foreach(f ${CR_SOURCES_})
    list(REMOVE_ITEM CR_SOURCES_${p} "${f}")
  endforeach()
  list(APPEND CR_HEADERS ${CR_HEADERS_})
  list(APPEND CR_SOURCES ${CR_SOURCES_})
endforeach()

# If building on linux add pthread and rt
if(UNIX AND NOT APPLE)
    set(CR_LINK_FLAGS pthread rt)
else()
    set(CR_LINK_FLAGS "")
endif()


# ---------------------------------------------------------------------
# Code coverage with lcov
# ---------------------------------------------------------------------

# Common configuration
add_compile_options(-Wall -pedantic)

# Code Coverage Configuration
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


# ---------------------------------------------------------------------
# Setup the library targets
# ---------------------------------------------------------------------
if ((NOT python AND NOT matlab AND NOT only-swig) OR all OR lib OR report)
  add_subdirectory(${PROJECT_SOURCE_DIR}/src)
endif()


# ---------------------------------------------------------------------
# Setup the test target
# ---------------------------------------------------------------------

# set CR_LIB_TARGETS
set(CR_LIB_TARGETS "")
if (full)
  set(CR_LIB_TARGETS CoreRobotics)
endif()
foreach(p ${CR_LIB_MODULES})
  list(APPEND CR_LIB_TARGETS ${CR_LIB_PREFIX}${p})
endforeach()

if ((tests OR all OR report) AND (GTest_FOUND))
  add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
endif()


# ---------------------------------------------------------------------
# Setup the python target
# ---------------------------------------------------------------------
# if(python OR all)
#     add_subdirectory(${PROJECT_SOURCE_DIR}/python)
# endif()


# ---------------------------------------------------------------------
# Setup the matlab target
# ---------------------------------------------------------------------
# if (matlab OR all)
#     add_subdirectory(${PROJECT_SOURCE_DIR}/matlab)
# endif()


# ---------------------------------------------------------------------
# Export the package configuration and version files
# ---------------------------------------------------------------------
if (NOT report)

  # export package for use from the build tree
  export (TARGETS ${CR_LIB_TARGETS} FILE ${PROJECT_BINARY_DIR}/CoreRoboticsTargets.cmake)
  export (PACKAGE CoreRobotics)

  # package definitions
  set (CONF_INCLUDE_DIRS ${CR_INCLUDE_DIRS})
  set (CONF_LIBRARIES    ${CR_LIB_TARGETS})
  set (CONF_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/lib)
  set (CONF_SOURCE_DIR   ${PROJECT_SOURCE_DIR})

  # package configuration and version files
  configure_file (CoreRoboticsConfig.cmake.in "${PROJECT_BINARY_DIR}/CoreRoboticsConfig.cmake" @ONLY)
  configure_file (CoreRoboticsConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/CoreRoboticsConfigVersion.cmake" @ONLY)
endif()


# ---------------------------------------------------------------------
# Setup the example targets
# ---------------------------------------------------------------------
if (examples OR all)
  add_subdirectory(${PROJECT_SOURCE_DIR}/examples)
endif()
