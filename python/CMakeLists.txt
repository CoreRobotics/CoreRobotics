# Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
# Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
# http://www.corerobotics.org

# If "python" is not a true consider it as a version number
if(NOT ${python} STREQUAL true)
  if(python EQUAL 3)
    SET(python_version 3)
  endif()
endif()

find_package (PythonInterp ${python_version} REQUIRED)
find_package (PythonLibs ${python_version} REQUIRED)
if (python_version EQUAL 3)
  if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    find_package(Boost COMPONENTS python37 REQUIRED)
  else ()
    find_package(Boost COMPONENTS python3 REQUIRED)
  endif ()
else ()
  find_package(Boost COMPONENTS python REQUIRED)
endif ()

message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")
message(STATUS "PYTHON_EXECUTABLE = ${PYTHON_EXECUTABLE}")
message(STATUS "PYTHON_INCLUDE_DIRS = ${PYTHON_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")

include_directories (
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/PyCoreRobotics.hpp)

# Get the wrapper modules
file(GLOB py_wrappers
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
)

set (PYTHON_TARGET "CoreRobotics")

# Add the python module
python_add_module (${PYTHON_TARGET} ${py_wrappers})

# Link the core robotics libraries
target_link_libraries(${PYTHON_TARGET}
  ${Boost_LIBRARIES} 
  ${PYTHON_LIBRARIES} 
  ${CR_LIB_TARGETS} 
  ${CR_LINK_FLAGS}
)

# Set the shared library output directory
set_target_properties(${PYTHON_TARGET} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/python
  LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/lib/python
  LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/lib/python)

# copy over the init
add_custom_command(TARGET ${PYTHON_TARGET} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
  ${PROJECT_SOURCE_DIR}/python/src/__init__.py
  ${PROJECT_SOURCE_DIR}/lib/python)

add_custom_command(TARGET ${PYTHON_TARGET} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${PROJECT_SOURCE_DIR}/python/examples
  ${PROJECT_SOURCE_DIR}/bin/)

add_custom_command(TARGET ${PYTHON_TARGET} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${PROJECT_SOURCE_DIR}/lib/python
  ${PROJECT_SOURCE_DIR}/bin/)