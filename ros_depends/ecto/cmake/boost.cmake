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
find_package(Boost COMPONENTS
  date_time
  python
  regex
  serialization
  system
  thread
  REQUIRED
  )

try_run(BOOST_VERSION_RUN_RESULT BOOST_VERSION_COMPILE_RESULT
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/boost-version.c
  COMPILE_DEFINITIONS -I${Boost_INCLUDE_DIRS}
  COMPILE_OUTPUT_VARIABLE BOOST_VERSION_COMPILE
  RUN_OUTPUT_VARIABLE Boost_VERSION
  )

if(NOT BOOST_VERSION_COMPILE_RESULT)
  message(FATAL_ERROR "Couldn't compile boost version checking program: ${BOOST_VERSION_COMPILE}")
endif()
message(STATUS "Boost version ${Boost_VERSION}")

macro(boost_feature_check checkname)
  if(NOT ${checkname}_CACHE)
    try_compile(${checkname}
      ${CMAKE_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${checkname}
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/boost_checks.cpp
      COMPILE_DEFINITIONS -I${Boost_INCLUDE_DIRS} -D${checkname}=1
      OUTPUT_VARIABLE ${checkname}_OUTPUT
      )
    message(STATUS "${checkname}: ${${checkname}}")
    if("${${checkname}_OUTPUT}" MATCHES ".*ECTO_CHECK_TRY_COMPILE_ERROR.*")
      message(FATAL_ERROR "Internal error when checking for boost feature ${checkname}")
    endif()
    set(${checkname}_CACHE TRUE CACHE BOOL "${checkname} cache variable" FORCE)
    mark_as_advanced(FORCE ${checkname}_CACHE)
  endif()
endmacro()

add_definitions(${Boost_DEFINITIONS})

boost_feature_check(ECTO_EXCEPTION_SHARED_POINTERS_ARE_CONST)
boost_feature_check(ECTO_EXCEPTION_DIAGNOSTIC_IMPL_TAKES_CHARSTAR)
boost_feature_check(ECTO_EXCEPTION_RELEASE_RETURNS_VOID)
boost_feature_check(ECTO_EXCEPTION_TAG_TYPE_NAME_RETURNS_STRING)
boost_feature_check(ECTO_EXCEPTION_TYPE_INFO_NESTED)
boost_feature_check(ECTO_EXCEPTION_CONTAINER_WITHOUT_CLONE)
if (NOT ECTO_EXCEPTION_WITHOUT_CLONE)
  set(ECTO_EXCEPTION_HAS_CLONE True)
endif()

configure_file(${ecto_SOURCE_DIR}/cmake/boost-config.hpp.in ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/boost-config.hpp)
