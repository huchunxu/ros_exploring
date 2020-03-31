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
if(${CMAKE_VERSION} VERSION_LESS 2.8.3)
  # The module defines the following variables:
  #   GIT_EXECUTABLE - path to git command line client
  #   GIT_FOUND - true if the command line client was found
  # Example usage:
  #   find_package(Git)
  #   if(GIT_FOUND)
  #     message("git found: ${GIT_EXECUTABLE}")
  #   endif()

  #=============================================================================
  # Copyright 2010 Kitware, Inc.
  #
  # Distributed under the OSI-approved BSD License (the "License");
  # see accompanying file Copyright.txt for details.
  #
  # This software is distributed WITHOUT ANY WARRANTY; without even the
  # implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  # See the License for more information.
  #=============================================================================
  # (To distribute this file outside of CMake, substitute the full
  #  License text for the above reference.)

  # Look for 'git' or 'eg' (easy git)
  #
  set(git_names git eg)

  # Prefer .cmd variants on Windows unless running in a Makefile
  # in the MSYS shell.
  #
  if(WIN32)
    if(NOT CMAKE_GENERATOR MATCHES "MSYS")
      set(git_names git.cmd git eg.cmd eg)
    endif()
  endif()

  find_program(GIT_EXECUTABLE
    NAMES ${git_names}
    DOC "git command line client"
    )
  mark_as_advanced(GIT_EXECUTABLE)

  # Handle the QUIETLY and REQUIRED arguments and set GIT_FOUND to TRUE if
  # all listed variables are TRUE

  find_package(PackageHandleStandardArgs)
  find_package_handle_standard_args(Git DEFAULT_MSG GIT_EXECUTABLE)

else()
  find_package(Git)
endif()

macro(git_status PROJECT)
  if (GIT_FOUND)
    execute_process(COMMAND ${GIT_EXECUTABLE} log -n1 --pretty=format:%H
      WORKING_DIRECTORY ${${PROJECT}_SOURCE_DIR}
      OUTPUT_VARIABLE ${PROJECT}_COMMITHASH
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE ${PROJECT}_COMMITHASH_STATUS
      )

    execute_process(COMMAND ${GIT_EXECUTABLE} log -n1 --pretty=format:%cD
      WORKING_DIRECTORY ${${PROJECT}_SOURCE_DIR}
      OUTPUT_VARIABLE ${PROJECT}_LAST_MODIFIED
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE ${PROJECT}_TREEHASH_STATUS
      )

    execute_process(COMMAND ${GIT_EXECUTABLE} describe --tags --dirty --always
      WORKING_DIRECTORY ${${PROJECT}_SOURCE_DIR}
      OUTPUT_VARIABLE ${PROJECT}_GITTAG_LONG
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE ${PROJECT}_GITTAG_STATUS
      )

    execute_process(COMMAND ${GIT_EXECUTABLE} describe --tags --abbrev=0
      WORKING_DIRECTORY ${${PROJECT}_SOURCE_DIR}
      OUTPUT_VARIABLE ${PROJECT}_GITTAG_SHORT
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE ${PROJECT}_GITTAG_STATUS
      )

  else()
    set(${PROJECT}_COMMITHASH treehash_unavailable)
    set(${PROJECT}_LAST_MODIFIED lastmod_unavailable)
    set(${PROJECT}_GITTAG   tag_unavailable)
  endif()

  message(STATUS "${PROJECT} commit:       ${${PROJECT}_COMMITHASH}")
  message(STATUS "${PROJECT} tag (long):   ${${PROJECT}_GITTAG_LONG}")
  message(STATUS "${PROJECT} tag (short):  ${${PROJECT}_GITTAG_SHORT}")
  message(STATUS "${PROJECT} last_mod:     ${${PROJECT}_LAST_MODIFIED}")


endmacro()


