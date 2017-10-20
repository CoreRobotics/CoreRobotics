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

# =====================================================================
# CMAKE Flags and variables
cmake_minimum_required(VERSION 3.0)

# Set CMAKE policy
cmake_policy(SET CMP0054 NEW)

# Require C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify the configuration
set(CMAKE_BUILD_TYPE Release)

# Define the project name and other CoreRobotics parameters
set(PROJECT_NAME "CoreRobotics")
set(LIB_TARGET_NAME ${PROJECT_NAME})
set(UTIL_TARGET_NAME "TestModules")

# project title
project( ${PROJECT_NAME} )

# =====================================================================
# 1. Setup the library target
#
# Include headers
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/core
    ${CMAKE_SOURCE_DIR}/src/math
    ${CMAKE_SOURCE_DIR}/src/models
    ${CMAKE_SOURCE_DIR}/src/physics
    ${CMAKE_SOURCE_DIR}/src/controllers
)

# Include 3rd part library headers
include_directories(
    ${CMAKE_SOURCE_DIR}/external/eigen/
)

# Header files
file(GLOB HEADERS_CORE
    src/core/*.hpp
)
file(GLOB HEADERS_MATH
    src/math/*.hpp
)
file(GLOB HEADERS_MODELS
    src/models/*.hpp
)
file(GLOB HEADERS_PHYSICS
    src/physics/*.hpp
)
file(GLOB HEADERS_CONTROLLERS
    src/controllers/*.hpp
)

# Source files
file(GLOB SOURCES_CORE
    src/core/*.cpp
)
file(GLOB SOURCES_MATH
    src/math/*.cpp
)
file(GLOB SOURCES_MODELS
    src/models/*.cpp
)
file(GLOB SOURCES_PHYSICS
    src/physics/*.cpp
)
file(GLOB SOURCES_CONTROLLERS
    src/controllers/*.cpp
)

if (NOT python OR all)

	# Create static (or shared) library
	add_library(${LIB_TARGET_NAME} STATIC "src/CoreRobotics.hpp" ${HEADERS_CORE} ${HEADERS_MATH} ${HEADERS_MODELS} ${HEADERS_PHYSICS} ${HEADERS_CONTROLLERS} ${SOURCES_CORE} ${SOURCES_MATH} ${SOURCES_MODELS} ${SOURCES_PHYSICS} ${SOURCES_CONTROLLERS})

	# Set the output directory
	set_target_properties( ${LIB_TARGET_NAME} PROPERTIES 
		LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
	)

	# Sort into a group structure
	source_group( "include" FILES "src/CoreRobotics.hpp")

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


	# =====================================================================
	# 2. Setup the utility target
	#
	# Source files
	file(GLOB UTILITIES
		utils/*.cpp
		utils/*.hpp
	)

	# Add an executable for the utilities
	add_executable( ${UTIL_TARGET_NAME} ${UTILITIES} )

	# Set the output directory
	set_target_properties( ${UTIL_TARGET_NAME} PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/utils/bin
	)

	# Add the library as a build dependency
	add_dependencies(${UTIL_TARGET_NAME} ${LIB_TARGET_NAME})

	# Link the library binary
	if(UNIX AND NOT APPLE) # This may be useful for apple too
		set(UTIL_LINK_FILES
			${CMAKE_SOURCE_DIR}/lib/${CMAKE_CFG_INTDIR}/$<TARGET_FILE_NAME:${PROJECT_NAME}>
			pthread
		)
	else()
		set(UTIL_LINK_FILES
			${CMAKE_SOURCE_DIR}/lib/${CMAKE_CFG_INTDIR}/$<TARGET_FILE_NAME:${PROJECT_NAME}>
		)
	endif()

	target_link_libraries( ${UTIL_TARGET_NAME} ${UTIL_LINK_FILES})

endif()
if(python OR all)

	if(WIN32)
		find_package(PythonInterp)
		execute_process(COMMAND
			"${PYTHON_EXECUTABLE}" -c
			"import numpy; print numpy.get_include()"
			OUTPUT_VARIABLE NUMPY_DIR)
		include_directories(${NUMPY_DIR})
	endif()

	find_package(SWIG REQUIRED)
	include(${SWIG_USE_FILE})

	if(NOT ${python} STREQUAL true)
		find_package(PythonLibs ${python} EXACT REQUIRED)
	else()
		find_package(PythonLibs)
	endif()
	include_directories(${PYTHON_INCLUDE_PATH})

	set(CMAKE_SWIG_FLAGS "")
	set_source_files_properties(${CMAKE_SOURCE_DIR}/python/swig/CoreRobotics.i PROPERTIES CPLUSPLUS ON)

	swig_add_module(${PROJECT_NAME} python
		${CMAKE_SOURCE_DIR}/python/swig/CoreRobotics.i
		${SOURCES_CORE} ${SOURCES_MATH} ${SOURCES_MODELS} ${SOURCES_PHYSICS} ${SOURCES_CONTROLLERS})

	set_target_properties(_${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/python/lib)
	set_target_properties(_${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/python/lib)
	set_target_properties(_${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/python/lib)

	swig_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})

	add_custom_command(TARGET _${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy
			${CMAKE_BINARY_DIR}/${PROJECT_NAME}.py
			${CMAKE_SOURCE_DIR}/python/lib/)
	add_custom_command(TARGET _${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory
			${CMAKE_SOURCE_DIR}/python/lib
			${CMAKE_SOURCE_DIR}/python/util)

endif()
