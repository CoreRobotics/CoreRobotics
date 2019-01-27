# =====================================================================
#
# Software License Agreement (BSD-3-Clause License)
# Copyright (c) 2019, CoreRobotics.
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


# build the CoreRobotics library ("CR" prefix files)
if (${full})
  add_library(CoreRobotics STATIC ${CR_SOURCES} ${CR_HEADERS})
  set_target_properties(CoreRobotics PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/CoreRobotics.hpp)

  # IMPORTANT: Add the library to the "export-set"
  install(TARGETS CoreRobotics
    EXPORT CoreRoboticsTargets
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
    ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib)
    PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}"
    COMPONENT dev)
endif()


# build the v2.0 shared modules
foreach(p ${CR_LIB_MODULES})

  add_library(${CR_LIB_PREFIX}${p} SHARED ${CR_HEADERS_${p}} ${CR_SOURCES_${p}})
  target_link_libraries(${CR_LIB_PREFIX}${p} ${CR_LIB_DEP_${p}})
  set_target_properties(${CR_LIB_PREFIX}${p} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/cr/${p})

  # IMPORTANT: Add the library to the "export-set"
  install(TARGETS ${CR_LIB_PREFIX}${p}
    EXPORT CoreRoboticsTargets
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
    ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib)
    PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/cr/"
    COMPONENT dev)

endforeach()
