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

# If "python" is not "true" consider it as a version number
# and find that Python version
if(NOT ${python} STREQUAL true AND NOT ${python} STREQUAL "only-swig")
    find_package(PythonLibs ${python} EXACT)
# Otherwise find the latest Python version installed
else()
    find_package(PythonLibs)
endif()

# Attempt to find SWIG
# find_package(SWIG)

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
            "import numpy as np; np.get_include()"
            OUTPUT_VARIABLE NUMPY_DIR)
        include_directories(${NUMPY_DIR})
    endif()

    # If SWIG not found or noswig set
    if(NOT SWIG_FOUND OR noswig)

        # Compile a shared library with the exsisting wrapper C++ file
        add_library(_${PROJECT_NAME} SHARED
            ${CR_HEADERS} ${CR_SOURCES}
            ${CMAKE_CURRENT_SOURCE_DIR}/swig/${PROJECT_NAME}PYTHON_wrap.cxx)

        # Link with Python
        target_link_libraries(_${PROJECT_NAME} ${PYTHON_LIBRARIES} ${LINK_FLAGS})

        # Rename to remove the lib prefix
        add_custom_command(TARGET _${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E rename
                ${CMAKE_CURRENT_SOURCE_DIR}/lib/$<TARGET_FILE_NAME:_${PROJECT_NAME}>
                ${CMAKE_CURRENT_SOURCE_DIR}/lib/_${PROJECT_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX})

    # If using SWIG
    else()

        # Include SWIG
        include(${SWIG_USE_FILE})

        # include the core robotics directories
        include_directories(${CR_INCLUDE_DIRS})

        message(STATUS ${CR_INCLUDE_DIRS})

        # Set SWIG to use C++
        set(CMAKE_SWIG_FLAGS "")
        set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/swig/CoreRobotics.i PROPERTIES CPLUSPLUS ON)

        # Create SWIG Module
        swig_add_module(${PROJECT_NAME} python
            ${CMAKE_CURRENT_SOURCE_DIR}/swig/CoreRobotics.i
            ${CR_HEADERS} ${CR_SOURCES})

        # Link Module with Python
        swig_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES} ${LINK_FLAGS})

        # Copy Python file to the Python lib folder
        add_custom_command(TARGET _${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.py
                ${CMAKE_CURRENT_SOURCE_DIR}/lib/)

        # Copy C++ wrapper file to the Python swig folder
        add_custom_command(TARGET _${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_BINARY_DIR}/${PROJECT_NAME}PYTHON_wrap.cxx
                ${CMAKE_CURRENT_SOURCE_DIR}/swig/)

    endif()

    # Set the shared library output directory
    set_target_properties(_${PROJECT_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib
        LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/lib
        LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/lib)

    # Copy the contents of the Python lib folder to the Python examples folder
    add_custom_command(TARGET _${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/lib
            ${CMAKE_CURRENT_SOURCE_DIR}/examples)
endif()
