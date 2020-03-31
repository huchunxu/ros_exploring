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
cmake_minimum_required(VERSION 2.8)

project(ecto)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

set(Python_ADDITIONAL_VERSIONS 2.7)
find_package(PythonLibs REQUIRED)


catkin_package(CFG_EXTRAS ecto-extras.cmake
               DEPENDS Boost PYTHON
               INCLUDE_DIRS include
               LIBRARIES ecto
)

# configure and install Python
catkin_python_setup()

set(${PROJECT_NAME}_GITHUB_URL https://github.com/plasmodic/ecto)

option(ECTO_TRACE_EXCEPTIONS "Trace ecto exceptions to stdout" OFF)

option(ECTO_STRESS_TEST "Stress testing of ecto" OFF)
if(ECTO_STRESS_TEST)
  message("ECTO_STRESS_TEST is ON")
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  option(ECTO_LOGGING "Ecto logging" ON)
else()
  option(ECTO_LOGGING "Ecto logging" OFF)
endif()

option(ECTO_WITH_INSTRUMENTATION "Instrument schedulers (for debugging)" OFF)

include(cmake/config.cmake)
include(cmake/boost.cmake)
include(cmake/version.cmake)
include(cmake/git.cmake)
include(cmake/ectoMacros.cmake)
include(cmake/doc.cmake)

git_status(ecto)

include_directories(include)
include_directories(src/lib)
include_directories(${CATKIN_DEVEL_PREFIX}/include)
include_directories(
  ${PYTHON_INCLUDE_PATH}
  ${Boost_INCLUDE_DIRS}
)

# deal with sources
set(ecto_LIBRARIES ecto)
add_subdirectory(src)
add_subdirectory(cmake)

set(ecto_SPHINX_DIR ${CMAKE_CURRENT_SOURCE_DIR}/python/ecto/sphinx)
ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/kitchen)

option(BUILD_DOC "Build docs, this will build tests and samples as well." ON)
if(BUILD_DOC)
  set(CATKIN_ENABLE_TESTING ON CACHE BOOL "Tests must be built if docs are expected to build." FORCE)
  set(BUILD_SAMPLES ON CACHE BOOL "Samples must be built if docs are expected to build." FORCE)
  add_subdirectory(doc)
#  find_package(catkin-sphinx QUIET)
  if (catkin-sphinx_FOUND)
      catkin_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source doc
              ${CMAKE_CURRENT_SOURCE_DIR}
              ${CMAKE_CURRENT_SOURCE_DIR}/samples
      )
  endif()
endif()

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

option(BUILD_SAMPLES "Build samples." ON)
if(BUILD_SAMPLES)
  add_subdirectory(samples)
endif()

# deal with doxygen and integration with sphinx
catkin_doxygen(ecto-doxygen "${CMAKE_CURRENT_SOURCE_DIR}/include;${CMAKE_CURRENT_SOURCE_DIR}/doc/doxygen")
add_custom_target(ecto-copy-doxygen-to-doc COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_BINARY_DIR}/ecto/api ${CMAKE_BINARY_DIR}/doc/html/ecto/api
                                           DEPENDS ecto-doxygen
)
add_dependencies(sphinx-doc ecto-copy-doxygen-to-doc)
