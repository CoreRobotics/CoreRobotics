# =====================================================================
#
# Software License Agreement (BSD-3-Clause License)
# Copyright (c) 2017, CoreRobotics.
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

# cmake settings
cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_policy(SET CMP0054 NEW)

# Define the project name and other CoreRobotics parameters
project(CoreRobotics)

# =====================================================================
# Find depdendencies
# =====================================================================

# If building wrappers Eigen 3.3.3 is required
set(EIGEN_VERSION 3.3.3)
find_package (Eigen3 ${EIGEN_VERSION})
find_package (Boost 1.48)
find_package (GTest)
find_package (OpenSceneGraph)

if(NOT Boost_INCLUDE_DIR)
     message(FATAL_ERROR "Boost_INCLUDE_DIR not found - please define the path to boost")
endif()

if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "EIGEN3_INCLUDE_DIR not found - please define the path to eigen")
else()
    message(STATUS "Eigen found at ${EIGEN3_INCLUDE_DIR}")
endif()

# If building on linux add pthread and rt
if(UNIX AND NOT APPLE) # This may be useful for apple too
    set(LINK_FLAGS pthread rt)
else()
    set(LINK_FLAGS "")
endif()


# =====================================================================
# Setup the library targets
# =====================================================================

# define the project names
set(LIB_TARGET_NAME CoreRobotics)

# include directories
file(GLOB CR_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
)

# add the subdirectory for the target
add_subdirectory(${PROJECT_SOURCE_DIR}/src/cr1)


# =====================================================================
# Export the package configuration and version files
# =====================================================================
# export package for use from the build tree
export (TARGETS ${LIB_TARGET_NAME} FILE ${PROJECT_BINARY_DIR}/CoreRoboticsTargets.cmake)
export (PACKAGE ${LIB_TARGET_NAME})

# package definitions
set (CONF_INCLUDE_DIRS ${CR_INCLUDE_DIRS})
set (CONF_LIBRARIES    ${LIB_TARGET_NAME})
set (CONF_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/lib)
set (CONF_SOURCE_DIR   ${PROJECT_SOURCE_DIR})

# package configuration and version files
configure_file (CoreRoboticsConfig.cmake.in "${PROJECT_BINARY_DIR}/CoreRoboticsConfig.cmake" @ONLY)
configure_file (CoreRoboticsConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/CoreRoboticsConfigVersion.cmake" @ONLY)


# =====================================================================
# Setup the example targets
# =====================================================================
add_subdirectory(${PROJECT_SOURCE_DIR}/examples/cr1)


# =====================================================================
# Setup the test target
# =====================================================================
add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
