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

# includes
include_directories(${CR_INCLUDE_DIRS})

# Header files
file(GLOB HEADERS_CORE ${CMAKE_CURRENT_SOURCE_DIR}/../core/*.hpp )
file(GLOB HEADERS_MATH ${CMAKE_CURRENT_SOURCE_DIR}/../math/*.hpp )
file(GLOB HEADERS_MODELS ${CMAKE_CURRENT_SOURCE_DIR}/../models/*.hpp )
file(GLOB HEADERS_PHYSICS ${CMAKE_CURRENT_SOURCE_DIR}/../physics/*.hpp )
file(GLOB HEADERS_CONTROLLERS ${CMAKE_CURRENT_SOURCE_DIR}/../controllers/*.hpp )

# Source files
file(GLOB SOURCES_CORE ${CMAKE_CURRENT_SOURCE_DIR}/../core/*.cpp )
file(GLOB SOURCES_MATH ${CMAKE_CURRENT_SOURCE_DIR}/../math/*.cpp )
file(GLOB SOURCES_MODELS ${CMAKE_CURRENT_SOURCE_DIR}/../models/*.cpp )
file(GLOB SOURCES_PHYSICS ${CMAKE_CURRENT_SOURCE_DIR}/../physics/*.cpp )
file(GLOB SOURCES_CONTROLLERS ${CMAKE_CURRENT_SOURCE_DIR}/../controllers/*.cpp )

# Create static (or shared) library
add_library(${LIB_TARGET_NAME} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/../CoreRobotics.hpp ${HEADERS_CORE} ${HEADERS_MATH} ${HEADERS_MODELS} ${HEADERS_PHYSICS} ${HEADERS_CONTROLLERS} ${SOURCES_CORE} ${SOURCES_MATH} ${SOURCES_MODELS} ${SOURCES_PHYSICS} ${SOURCES_CONTROLLERS})

set_target_properties(${LIB_TARGET_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
)

# Sort into a group structure
source_group( "include" FILES ${CMAKE_CURRENT_SOURCE_DIR}/../CoreRobotics.hpp)
source_group( "include\\core" FILES ${HEADERS_CORE})
source_group( "include\\math" FILES ${HEADERS_MATH})
source_group( "include\\models" FILES ${HEADERS_MODELS})
source_group( "include\\physics" FILES ${HEADERS_PHYSICS})
source_group( "include\\controllers" FILES ${HEADERS_CONTROLLERS})
source_group( "src\\core" FILES ${SOURCES_CORE})
source_group( "src\\math" FILES ${SOURCES_MATH})
source_group( "src\\models" FILES ${SOURCES_MODELS})
source_group( "src\\physics" FILES ${SOURCES_PHYSICS})
source_group( "src\\controllers" FILES ${SOURCES_CONTROLLERS})

