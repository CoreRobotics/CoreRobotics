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

# Get the example name and source
file(GLOB source *.cpp)
get_filename_component(example ${source} NAME_WE)

if(OpenSceneGraph_FOUND)
  find_package (OpenSceneGraph 3.6 REQUIRED osgDB osgUtil osg osgViewer osgGA osgShadow)
  add_definitions(-DUSE_OSG) # macro to use OSG
endif()

# create the example target
set(EXAMPLE_TARGET_NAME ${CR_LIB_PREFIX}${example})
file(GLOB CR_EXAMPLE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_executable(${EXAMPLE_TARGET_NAME} ${CR_EXAMPLE_SOURCES})
add_dependencies(${EXAMPLE_TARGET_NAME} ${CR_LIB_TARGETS})
target_link_libraries(${EXAMPLE_TARGET_NAME} ${CR_LIB_TARGETS} ${OPENSCENEGRAPH_LIBRARIES} ${CR_LINK_FLAGS} ${GTEST_LIBRARIES})
set_target_properties(${EXAMPLE_TARGET_NAME}
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
