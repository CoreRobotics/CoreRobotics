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

# This may be useful for apple too
if(UNIX AND NOT APPLE)
    set(CR_LINK_FLAGS pthread rt)
else()
    set(CR_LINK_FLAGS "")
endif()

# create the test target
set(TEST_TARGET_NAME ${CR_LIB_PREFIX}tests)
file(GLOB CR_TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

if (NOT full)
  file(GLOB FILES ${CMAKE_CURRENT_SOURCE_DIR}/test_CR*.cpp)
  foreach(f ${FILES})
    list(REMOVE_ITEM CR_TEST_SOURCES "${f}")
  endforeach()
endif()
add_executable(${TEST_TARGET_NAME} ${CR_TEST_SOURCES})
add_dependencies(${TEST_TARGET_NAME} ${CR_LIB_TARGETS})
target_link_libraries(${TEST_TARGET_NAME} ${CR_LIB_TARGETS} ${CR_LINK_FLAGS} ${GTEST_LIBRARIES})
set_target_properties(${TEST_TARGET_NAME}
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)

# IMPORTANT: Add the executable to the "export-set"
install(TARGETS ${TEST_TARGET_NAME}
  EXPORT CoreRoboticsTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
