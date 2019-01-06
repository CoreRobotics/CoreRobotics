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

# Find CoreRobotics
find_package(CoreRobotics REQUIRED)
include_directories(${CR_INCLUDE_DIRS})
link_directories(${CR_LIBRARY_DIRS})

# Find GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# set the test target name
set(TEST_TARGET_NAME TestModules)

# Setup the test target
file(GLOB CR_TESTS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_executable(${TEST_TARGET_NAME} ${CR_TESTS})
add_dependencies(${TEST_TARGET_NAME} ${CR_LIBRARIES})
target_link_libraries(${TEST_TARGET_NAME} ${CR_LIBRARIES} ${LINK_FLAGS} ${GTEST_LIBRARIES})
set_target_properties(${TEST_TARGET_NAME}
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)
