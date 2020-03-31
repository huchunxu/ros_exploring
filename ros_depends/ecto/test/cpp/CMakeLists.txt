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

if(NOT CATKIN_ENABLE_TESTING)
  message(STATUS "Disabling testing since gtest was not found.")
  return()
endif()

include_directories(${GTEST_INCLUDE_DIRS}
                    ${PYTHON_INCLUDE_PATH}
                    ${Boost_INCLUDE_DIRS}
)

catkin_add_gtest(ecto-test
  main.cpp
  tendril.cpp
  tendrils.cpp
  spore.cpp
  exceptions.cpp
  graph.cpp
  profile.cpp
  serialization.cpp
  strands.cpp
  scheduler.cpp
  clone.cpp
  static.cpp
  )

target_link_libraries(ecto-test
  ecto
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

# This is actually used in the plasm loader Python test file
add_executable(plasm_loader
  plasm_loader.cpp
)

target_link_libraries(plasm_loader
  ecto
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)
