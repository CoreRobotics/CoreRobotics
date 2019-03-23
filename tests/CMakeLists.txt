# Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
# Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
# http://www.corerobotics.org


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

# pthread needs to come after gtest
target_link_libraries(${TEST_TARGET_NAME} ${CR_LIB_TARGETS} ${GTEST_LIBRARIES} ${CR_LINK_FLAGS})
set_target_properties(${TEST_TARGET_NAME}
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)

# Include code-coverage settings:
if (report)
  target_link_libraries(${TEST_TARGET_NAME} coverage_config)
  add_test(NAME cr_test COMMAND ${TEST_TARGET_NAME})
endif()

# IMPORTANT: Add the executable to the "export-set"
if(NOT report)
  install(TARGETS ${TEST_TARGET_NAME}
    EXPORT CoreRoboticsTargets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
endif()

if(format)
  set_target_properties(
    ${TEST_TARGET_NAME} PROPERTIES
    CXX_CLANG_FORMAT "${DO_CLANG_FORMAT}"
  )
endif()

if(lint)
  set_target_properties(
    ${TEST_TARGET_NAME} PROPERTIES
    CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
  )
endif()
