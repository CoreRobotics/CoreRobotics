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
find_package (pybind11 REQUIRED)

message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")
message(STATUS "PYTHON_EXECUTABLE = ${PYTHON_EXECUTABLE}")
message(STATUS "PYTHON_INCLUDE_DIRS = ${PYTHON_INCLUDE_DIRS}")

include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src)

# Get the wrapper modules
file(GLOB py_wrappers
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
)

set (PYTHON_TARGET "CoreRobotics")

# Add the python module
pybind11_add_module (${PYTHON_TARGET} ${py_wrappers})

# Link the CoreRobotics libraries
target_link_libraries(${PYTHON_TARGET}
  PRIVATE ${CR_LIB_TARGETS} ${CR_LINK_FLAGS}
)

# Set the shared library output directory
set_target_properties(${PYTHON_TARGET} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/python
  LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/lib/python
  LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/lib/python)

add_custom_command(TARGET ${PYTHON_TARGET} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${PROJECT_SOURCE_DIR}/python/examples
  ${PROJECT_SOURCE_DIR}/bin/)

add_custom_command(TARGET ${PYTHON_TARGET} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${PROJECT_SOURCE_DIR}/lib/python
  ${PROJECT_SOURCE_DIR}/bin/)

install(TARGETS ${PYTHON_TARGET}
  EXPORT CoreRoboticsTargets
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}"
  COMPONENT dev)
