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

    # Copy the lib folder to the examples folder
    add_custom_command(TARGET ${Matlab_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/matlab/lib
            ${CMAKE_SOURCE_DIR}/matlab/examples)
endif()
