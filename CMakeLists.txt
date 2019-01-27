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

#
# Settings:
#   -Dall = [true,false]            all targets
#   -Dsplit = [true,false]          split up CoreRobotics libraries
#   -Dfull = [true,false]           use only 2.0 source
#   -Dlib = [true,false]            c++ lib
#   -Dtests = [true,false]          c++ tests
#   -Dpython = [true,false,2,3]     python binding

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

# define modules and target prefix
set(CR_LIB_MODULES control core math model noise physics signal world)
set(CR_LIB_PREFIX "cr-")

# create individual header and source files
set(CR_HEADERS_1 "")
set(CR_SOURCES_1 "")
foreach(p ${CR_LIB_MODULES})
  file(GLOB CR_HEADERS_1_${p} ${CMAKE_CURRENT_SOURCE_DIR}/src/${p}/CR*.hpp)
  file(GLOB CR_SOURCES_1_${p} ${CMAKE_CURRENT_SOURCE_DIR}/src/${p}/CR*.cpp)
  file(GLOB CR_HEADERS_2_${p} ${CMAKE_CURRENT_SOURCE_DIR}/src/${p}/*.hpp)
  file(GLOB CR_SOURCES_2_${p} ${CMAKE_CURRENT_SOURCE_DIR}/src/${p}/*.cpp)
  foreach(f ${CR_HEADERS_1_${p}})
    list(REMOVE_ITEM CR_HEADERS_2_${p} "${f}")
  endforeach()
  foreach(f ${CR_SOURCES_1_${p}})
    list(REMOVE_ITEM CR_SOURCES_2_${p} "${f}")
  endforeach()
  list(APPEND CR_HEADERS_1 ${CR_HEADERS_1_${p}})
  list(APPEND CR_SOURCES_1 ${CR_SOURCES_1_${p}})
  list(APPEND CR_HEADERS_2 ${CR_HEADERS_2_${p}})
  list(APPEND CR_SOURCES_2 ${CR_SOURCES_2_${p}})
endforeach()

# If building on linux add pthread and rt
if(UNIX AND NOT APPLE) # This may be useful for apple too
    set(CR_LINK_FLAGS pthread rt)
else()
    set(CR_LINK_FLAGS "")
endif()


# ---------------------------------------------------------------------
# Setup the library targets
# ---------------------------------------------------------------------

# build the CoreRobotics library
add_library(CoreRobotics STATIC ${CR_SOURCES_1} ${CR_HEADERS_1})
set_target_properties(CoreRobotics PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/src/CoreRobotics.hpp)

# build the v2.0 shared modules
add_library(${CR_LIB_PREFIX}core SHARED ${CR_HEADERS_2_core} ${CR_SOURCES_2_core})
set_target_properties(${CR_LIB_PREFIX}core PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/src/cr/core)

add_library(${CR_LIB_PREFIX}signal SHARED ${CR_HEADERS_2_signal} ${CR_SOURCES_2_signal})
target_link_libraries(${CR_LIB_PREFIX}signal cr-core) # this is the trick
set_target_properties(${CR_LIB_PREFIX}signal PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/src/cr/signal)

add_library(${CR_LIB_PREFIX}math SHARED ${CR_HEADERS_2_math} ${CR_SOURCES_2_math})
set_target_properties(${CR_LIB_PREFIX}math PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/src/cr/math)

add_library(${CR_LIB_PREFIX}model SHARED ${CR_HEADERS_2_model} ${CR_SOURCES_2_model})
target_link_libraries(${CR_LIB_PREFIX}model cr-core cr-math) # this is the trick
set_target_properties(${CR_LIB_PREFIX}model PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/src/cr/model)

add_library(${CR_LIB_PREFIX}noise SHARED ${CR_HEADERS_2_noise} ${CR_SOURCES_2_noise})
target_link_libraries(${CR_LIB_PREFIX}noise cr-math) # this is the trick
set_target_properties(${CR_LIB_PREFIX}noise PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/src/cr/noise)

add_library(${CR_LIB_PREFIX}physics SHARED ${CR_HEADERS_2_physics} ${CR_SOURCES_2_physics})
set_target_properties(${CR_LIB_PREFIX}physics PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/src/cr/physics)

add_library(${CR_LIB_PREFIX}world SHARED ${CR_HEADERS_2_world} ${CR_SOURCES_2_world})
target_link_libraries(${CR_LIB_PREFIX}world cr-core cr-physics) # this is the trick
set_target_properties(${CR_LIB_PREFIX}world PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/src/cr/world)

add_library(${CR_LIB_PREFIX}control SHARED ${CR_HEADERS_2_control} ${CR_SOURCES_2_control})
target_link_libraries(${CR_LIB_PREFIX}control cr-core) # this is the trick
set_target_properties(${CR_LIB_PREFIX}control PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
  PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/src/cr/control)




# foreach(p ${CR_LIB_MODULES})
#  add_library(${CR_LIB_PREFIX}${p} STATIC ${CR_HEADERS_2_${p}} ${CR_SOURCES_2_${p}})
#  set_target_properties(${CR_LIB_PREFIX}${p} PROPERTIES
#    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
#    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
#    PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/src/cr/${p})
# endforeach()


# define the project names
# set(LIB_TARGET_NAME CoreRobotics)

# If "all" build c++ static library
# If "python" or "matlab" do not build c++ library
# If "all" and python version specified build c++ library
# if ((NOT python AND NOT matlab AND NOT only-swig) OR all OR lib)
    # if "split", then multiple libraries will be created
#    if (split)
#        set(CR_LIB_TARGETS "")
#        foreach(p ${CR_LIB_MODULES})
#            list(APPEND CR_LIB_TARGETS ${CR_LIB_PREFIX}${p})
#        endforeach()
#        add_subdirectory(${PROJECT_SOURCE_DIR}/src)
    # otherwise we just make "CoreRobotics"
#    else()
#        set(CR_LIB_TARGETS ${PROJECT_NAME})
#        add_library(${CR_LIB_TARGETS} ${CR_SOURCES} ${CR_HEADERS})
#        set_target_properties(${CR_LIB_TARGETS} PROPERTIES
#            LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
#            ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
#            PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/src/CoreRobotics.hpp)
#    endif()
#endif()


# ---------------------------------------------------------------------
# Setup the test target
# ---------------------------------------------------------------------
# if ((tests OR all) AND (GTest_FOUND))
#     add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
# endif()

# set CR_LIB_TARGETS
set(CR_LIB_TARGETS CoreRobotics)
foreach(p ${CR_LIB_MODULES})
  list(APPEND CR_LIB_TARGETS ${CR_LIB_PREFIX}${p})
endforeach()

# tests
set(TEST_TARGET_NAME ${CR_LIB_PREFIX}tests)
file(GLOB CR_TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
add_executable(${TEST_TARGET_NAME} ${CR_TEST_SOURCES})
add_dependencies(${TEST_TARGET_NAME} ${CR_LIB_TARGETS})
target_link_libraries(${TEST_TARGET_NAME} ${CR_LIB_TARGETS} ${CR_LINK_FLAGS} ${GTEST_LIBRARIES})


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
# export package for use from the build tree
# export (TARGETS ${CR_LIB_TARGETS} FILE ${PROJECT_BINARY_DIR}/CoreRoboticsTargets.cmake)
# export (PACKAGE CoreRobotics)

# package definitions
# set (CONF_INCLUDE_DIRS ${CR_INCLUDE_DIRS})
# set (CONF_LIBRARIES    ${CR_LIB_TARGETS})
# set (CONF_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/lib)
# set (CONF_SOURCE_DIR   ${PROJECT_SOURCE_DIR})

# package configuration and version files
# configure_file (CoreRoboticsConfig.cmake.in "${PROJECT_BINARY_DIR}/CoreRoboticsConfig.cmake" @ONLY)
# configure_file (CoreRoboticsConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/CoreRoboticsConfigVersion.cmake" @ONLY)


# ---------------------------------------------------------------------
# Setup the example targets
# ---------------------------------------------------------------------
# if (examples OR all)
#     add_subdirectory(${PROJECT_SOURCE_DIR}/examples)
# endif()
