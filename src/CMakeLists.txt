# Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
# Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
# http://www.corerobotics.org


# build the CoreRobotics library ("CR" prefix files)
if (${full})

  set(target CoreRobotics)

  add_library(${target} STATIC ${CR_SOURCES} ${CR_HEADERS})
  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/CoreRobotics.hpp)

  # IMPORTANT: Add the library to the "export-set"
  if (NOT report)
  install(TARGETS ${target}
    EXPORT CoreRoboticsTargets
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
    ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
    PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}"
    COMPONENT dev)
  endif()

  # Include code-coverage settings:
  if (report)
     target_link_libraries(${target} coverage_config)
  endif()
endif()


# build the v2.0 shared modules
foreach(p ${CR_LIB_MODULES})

  set(target ${CR_LIB_PREFIX}${p})

  add_library(${target} SHARED ${CR_HEADERS_${p}} ${CR_SOURCES_${p}})
  target_link_libraries(${target} ${CR_LIB_DEP_${p}} ${CR_LINK_FLAGS})
  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/cr/${p}
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    COMPILE_FLAGS "${WARNING_FLAGS}"
  )

  if(lint)
    set_target_properties(
      ${target} PROPERTIES
      CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
    )
  endif()

  # IMPORTANT: Add the library to the "export-set"
  if (NOT report)
    install(TARGETS ${target}
      EXPORT CoreRoboticsTargets
      LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
      ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
      PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/cr/"
      COMPONENT dev)
  endif()

  # Include code-coverage settings:
  if (report)
    target_link_libraries(${target} coverage_config)
  endif()

endforeach()
