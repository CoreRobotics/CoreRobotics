# Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
# Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
# http://www.corerobotics.org

# Include SWIG
include(${SWIG_USE_FILE})

# include the core robotics directories
include_directories(${CR_INCLUDE_DIRS})

# Set SWIG to use C++
set_source_files_properties(
  SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/swig/CoreRobotics.i
  PROPERTIES CPLUSPLUS ON)

# Create SWIG Module
swig_add_library(${PROJECT_NAME}
  LANGUAGE python
  SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/swig/CoreRobotics.i)

# Link Module with Python
swig_link_libraries(${PROJECT_NAME}
  ${PYTHON_LIBRARIES}
  ${Python3_LIBRARIES}
  ${CR_LIB_TARGETS})

# Copy Python file to the Python lib folder
message(STATUS ${CMAKE_BINARY_DIR})
add_custom_command(TARGET _${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
  ${CMAKE_BINARY_DIR}/python/${PROJECT_NAME}.py
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/)

# Copy C++ wrapper file to the Python swig folder
# add_custom_command(TARGET _${PROJECT_NAME} POST_BUILD
#  COMMAND ${CMAKE_COMMAND} -E copy
#    ${CMAKE_BINARY_DIR}/${PROJECT_NAME}PYTHON_wrap.cxx
#    ${CMAKE_CURRENT_SOURCE_DIR}/swig/)

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
