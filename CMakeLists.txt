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

# Find the 3rd party libraries

# If building wrappers Eigen 3.3.3 is required
set(EIGEN_VERSION 3.3.3)
find_package (Eigen3 ${EIGEN_VERSION})
find_package (Boost 1.48)

if(NOT Boost_INCLUDE_DIR)
    message(FATAL_ERROR "Boost_INCLUDE_DIR not found - please define the path to boost")
endif()

if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "EIGEN3_INCLUDE_DIR not found - please define the path to eigen")
else()
    message(STATUS ${EIGEN3_INCLUDE_DIR})
endif()

# Include headers
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/core
    ${CMAKE_SOURCE_DIR}/src/math
    ${CMAKE_SOURCE_DIR}/src/models
    ${CMAKE_SOURCE_DIR}/src/physics
    ${CMAKE_SOURCE_DIR}/src/controllers
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
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

# If building on linux add pthread and rt
if(UNIX AND NOT APPLE) # This may be useful for apple too
	set(LINK_FLAGS pthread rt)
else()
	set(LINK_FLAGS "")
endif()

# If "all" build c++ static library
# If "python" or "matlab" do not build c++ library
# If "all" and python version specified build c++ library
if ((NOT python AND NOT matlab AND NOT only-swig) OR all OR cpp)

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
		RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
	)

	# Add the library as a build dependency
	add_dependencies(${UTIL_TARGET_NAME} ${LIB_TARGET_NAME})

	# Set libraries to link
	set(UTIL_LINK_FILES
		${CMAKE_SOURCE_DIR}/lib/${CMAKE_CFG_INTDIR}/$<TARGET_FILE_NAME:${PROJECT_NAME}>
	)

	# Link the library binary
	target_link_libraries( ${UTIL_TARGET_NAME} ${UTIL_LINK_FILES} ${LINK_FLAGS})

endif()

# If "python" or "all" build the Python library
if(python OR all)

	# If "python" is not "true" consider it as a version number
	# and find that Python version
	if(NOT ${python} STREQUAL true AND NOT ${python} STREQUAL "only-swig")
		find_package(PythonLibs ${python} EXACT)
	# Otherwise find the latest Python version installed
	else()
		find_package(PythonLibs)
	endif()

	# Attempt to find SWIG
	find_package(SWIG)

	# If python set as only-swig generate the wrappers but don't build
	if(${python} MATCHES "only-swig")

		# Include SWIG
		include(${SWIG_USE_FILE})

		# Get the Include Directories
		get_property(INCLUDE_DIRECTORIES DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

		# Create a list of arguments to pass to swig by appending -I in front of each path
		set(PYTHON_SWIG_INCLUDE_ARGS "")
		foreach(dir ${INCLUDE_DIRECTORIES})
			list(APPEND PYTHON_SWIG_INCLUDE_ARGS -I${dir})
		endforeach()

		# Add a custom target to generate the wrapper files
		add_custom_target(PythonWrappers ALL
			${SWIG_EXECUTABLE} -c++ -python
				-outdir ${CMAKE_SOURCE_DIR}/python/lib
				-o ${CMAKE_SOURCE_DIR}/python/swig/CoreRoboticsPYTHON_wrap.cxx
				${PYTHON_SWIG_INCLUDE_ARGS}
				${CMAKE_SOURCE_DIR}/python/swig/CoreRobotics.i)

	# Check if Python was found and skip if not found
	elseif(PythonLibs_FOUND)

		# Add Python's include path
		include_directories(${PYTHON_INCLUDE_PATH})
		
		# If on Windows use Python to get the Numpy include directory
		if(WIN32 OR APPLE)
			find_package(PythonInterp)
			execute_process(COMMAND
				"${PYTHON_EXECUTABLE}" -c
				"import numpy; print numpy.get_include()"
				OUTPUT_VARIABLE NUMPY_DIR)
			include_directories(${NUMPY_DIR})
		endif()

		# If SWIG not found or noswig set
		if(NOT SWIG_FOUND OR noswig)

			# Compile a shared library with the exsisting wrapper C++ file
			add_library(_${PROJECT_NAME} SHARED
				${SOURCES_CORE} ${SOURCES_MATH} ${SOURCES_MODELS} ${SOURCES_PHYSICS} ${SOURCES_CONTROLLERS}
				${CMAKE_SOURCE_DIR}/python/swig/${PROJECT_NAME}PYTHON_wrap.cxx)

			# Link with Python
			target_link_libraries(_${PROJECT_NAME} ${PYTHON_LIBRARIES} ${LINK_FLAGS})

			# Rename to remove the lib prefix
			add_custom_command(TARGET _${PROJECT_NAME} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E rename
					${CMAKE_SOURCE_DIR}/python/lib/$<TARGET_FILE_NAME:_${PROJECT_NAME}>
					${CMAKE_SOURCE_DIR}/python/lib/_${PROJECT_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX})

		# If using SWIG
		else()

			# Include SWIG
			include(${SWIG_USE_FILE})

			# Set SWIG to use C++
			set(CMAKE_SWIG_FLAGS "")
			set_source_files_properties(${CMAKE_SOURCE_DIR}/python/swig/CoreRobotics.i PROPERTIES CPLUSPLUS ON)

			# Create SWIG Module
			swig_add_module(${PROJECT_NAME} python
				${CMAKE_SOURCE_DIR}/python/swig/CoreRobotics.i
				${SOURCES_CORE} ${SOURCES_MATH} ${SOURCES_MODELS} ${SOURCES_PHYSICS} ${SOURCES_CONTROLLERS})

			# Link Module with Python
			swig_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES} ${LINK_FLAGS})

			# Copy Python file to the Python lib folder
			add_custom_command(TARGET _${PROJECT_NAME} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy
					${CMAKE_BINARY_DIR}/${PROJECT_NAME}.py
					${CMAKE_SOURCE_DIR}/python/lib/)

			# Copy C++ wrapper file to the Python swig folder
			add_custom_command(TARGET _${PROJECT_NAME} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy
					${CMAKE_BINARY_DIR}/${PROJECT_NAME}PYTHON_wrap.cxx
					${CMAKE_SOURCE_DIR}/python/swig/)

		endif()

		# Set the shared library output directory
		set_target_properties(_${PROJECT_NAME} PROPERTIES
			LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/python/lib
			LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/python/lib
			LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/python/lib)

		# Copy the contents of the Python lib folder to the Python util folder
		add_custom_command(TARGET _${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory
				${CMAKE_SOURCE_DIR}/python/lib
				${CMAKE_SOURCE_DIR}/python/util)
	endif()
endif()

# If "matlab" or "all" build the Matlab library
if(matlab OR all)

	# Find Matlab
	find_package(Matlab OPTIONAL_COMPONENTS MX_LIBRARY)

	# Include the Matlab src directory
	include_directories(${CMAKE_SOURCE_DIR}/matlab/src)

	# Attempt to find SWIG
	find_package(SWIG)

	# If SWIG was found test for Matlab support
	if(SWIG_FOUND)
		# Read the SWIG help output
		execute_process(COMMAND ${SWIG_EXECUTABLE} -help
			OUTPUT_VARIABLE swig_help)
		# Check if includes the word matlab
		if(${swig_help} MATCHES matlab)
			set(SWIG_Matlab_FOUND true)
		endif()
	endif()

	# If only-swig is defined create the wrapper scripts but don't build
	if(${matlab} MATCHES "only-swig" AND SWIG_FOUND)

		# Get the Include Directories
		get_property(INCLUDE_DIRECTORIES DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

		# Create a list of arguments to pass to swig by appending -I in front of each path
		set(Matlab_SWIG_INCLUDE_ARGS "")
		foreach(dir ${INCLUDE_DIRECTORIES})
			list(APPEND Matlab_SWIG_INCLUDE_ARGS -I${dir})
		endforeach()

		# Add a custom target to generate the wrapper files
		add_custom_target(MatlabWrappers ALL
			${SWIG_EXECUTABLE} -c++ -matlab
				-outdir ${CMAKE_SOURCE_DIR}/matlab/lib
				-o ${CMAKE_SOURCE_DIR}/matlab/swig/CoreRoboticsMATLAB_wrap.cxx
				${Matlab_SWIG_INCLUDE_ARGS}
				${CMAKE_SOURCE_DIR}/matlab/swig/CoreRobotics.i)

	# If Matlab was found
	elseif(Matlab_FOUND AND Matlab_MX_LIBRARY)

		# Include the Matlab directories
		include_directories(${Matlab_INCLUDE_DIRS})

		# Set the Matlab project name
		set(Matlab_PROJECT_NAME ${PROJECT_NAME}Matlab)

		# If building on Windows remove the threading library from Core
		if(WIN32)
			list(REMOVE_ITEM SOURCES_CORE ${CMAKE_SOURCE_DIR}/src/core/CRThread.cpp)
		endif()

		# If no Matlab support or noswig defined
		if(noswig OR NOT SWIG_Matlab_FOUND)

			# Compile a shared library with the existing C++ wrapper files
			add_library(${Matlab_PROJECT_NAME} SHARED
				${SOURCES_CORE} ${SOURCES_MATH} ${SOURCES_MODELS} ${SOURCES_PHYSICS} ${SOURCES_CONTROLLERS}
				${CMAKE_SOURCE_DIR}/matlab/swig/${PROJECT_NAME}MATLAB_wrap.cxx)

			# Link with Matlab
			target_link_libraries(${Matlab_PROJECT_NAME} ${Matlab_LIBRARIES} ${LINK_FLAGS})

		# If using SWIG
		else()

			# Include SWIG
			include(${SWIG_USE_FILE})

			# Set SWIG to use C++
			set(CMAKE_SWIG_FLAGS "")
			set_source_files_properties(${CMAKE_SOURCE_DIR}/matlab/swig/CoreRobotics.i PROPERTIES CPLUSPLUS ON)

			# Create SWIG Module
			swig_add_module(${Matlab_PROJECT_NAME} matlab
				${CMAKE_SOURCE_DIR}/matlab/swig/CoreRobotics.i
				${SOURCES_CORE} ${SOURCES_MATH} ${SOURCES_MODELS} ${SOURCES_PHYSICS} ${SOURCES_CONTROLLERS})

			# Link Module with Matlab
			swig_link_libraries(${Matlab_PROJECT_NAME} ${Matlab_LIBRARIES} ${LINK_FLAGS})

			# Set a list of files to copy to the Matlab lib directory
			set(matlab_files SwigGet.m SwigMem.m SwigRef.m)

			# Copy each file to the lib directory
			foreach(matlab_file ${matlab_files})
				add_custom_command(TARGET ${Matlab_PROJECT_NAME} POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy
						${CMAKE_BINARY_DIR}/${matlab_file}
						${CMAKE_SOURCE_DIR}/matlab/lib)
			endforeach()

			# Copy the Matlab package folder to the lib directory
			add_custom_command(TARGET ${Matlab_PROJECT_NAME} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_directory
					${CMAKE_BINARY_DIR}/+${PROJECT_NAME}
					${CMAKE_SOURCE_DIR}/matlab/lib/+${PROJECT_NAME})

			# Copy the C++ wrapper code to the Matlab swig folder for compiling without SWIG
			add_custom_command(TARGET ${Matlab_PROJECT_NAME} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy
					${CMAKE_BINARY_DIR}/${PROJECT_NAME}MATLAB_wrap.cxx
					${CMAKE_SOURCE_DIR}/matlab/swig)

		endif()

		# Set the library output directory
		set_target_properties(${Matlab_PROJECT_NAME} PROPERTIES
			LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
			LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}
			LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR})

		# Copy the shared library to the lib directory and rename
		add_custom_command(TARGET ${Matlab_PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy
				${CMAKE_BINARY_DIR}/$<TARGET_FILE_NAME:${Matlab_PROJECT_NAME}>
				${CMAKE_SOURCE_DIR}/matlab/lib/${PROJECT_NAME}MEX.${Matlab_MEX_EXTENSION})

		# Copy the lib folder to the util folder
		add_custom_command(TARGET ${Matlab_PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory
				${CMAKE_SOURCE_DIR}/matlab/lib
				${CMAKE_SOURCE_DIR}/matlab/util)
	endif()
endif()
