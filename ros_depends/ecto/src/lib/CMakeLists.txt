#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#this helps with visual studio projects.
file(GLOB ecto_HEADERS
  ${CMAKE_SOURCE_DIR}/include/ecto/*.hpp
  ${CMAKE_SOURCE_DIR}/include/ecto/*/*.hpp
  )
source_group("Headers" FILES ${ecto_HEADERS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(ecto SHARED
  abi.cpp
  cell.cpp
  edge.cpp
  tendril.cpp
  tendrils.cpp
  plasm.cpp
  plasm/impl.cpp
  util.cpp
  log.cpp
  except.cpp
  parameters.cpp
  profile.cpp
  python.cpp
  registry.cpp
  rethrow.cpp
  serialization.cpp
  scheduler.cpp
  graph/utilities.cpp
  strand.cpp
  test.cpp
  time.cpp
  ${ecto_HEADERS}
  )

target_link_libraries(ecto
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

set_source_files_properties(log.cpp
  PROPERTIES
  COMPILE_FLAGS -DSOURCE_DIR=\\"${ecto_SOURCE_DIR}\\"
  )

if(UNIX)
  set_target_properties(ecto PROPERTIES
    COMPILE_FLAGS "${FASTIDIOUS_FLAGS}"
    VERSION ${ecto_VERSION}
    SOVERSION ${ECTO_SOVERSION}
    )
elseif(WIN32)
  set_target_properties(ecto PROPERTIES
    COMPILE_FLAGS "${FASTIDIOUS_FLAGS}"
    VERSION ${ecto_VERSION}
    SOVERSION ${ECTO_SOVERSION}
    OUTPUT_NAME ecto_cpp
    )
endif()

#install the main shared lib
install(TARGETS ecto
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        COMPONENT main
)
