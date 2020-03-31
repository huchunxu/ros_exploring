#
# Copyright (c) 2011, Willow Garage, Inc.
#  All rights reserved.
# 
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#      * Neither the name of the Willow Garage, Inc. nor the names of its
#        contributors may be used to endorse or promote products derived from
#        this software without specific prior written permission.
# 
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
# 
if(NOT ecto_SOURCE_DIR)
  #not doing a standalone test project, so don't do these types of tests.
  return()
endif()

option(ECTO_ENABLE_COMPILE_TESTS "Enable compile tests... only need to do this occasionally" 
  OFF)

if (ECTO_ENABLE_COMPILE_TESTS)
  macro(add_compile_fail_test FILENAME)
    add_library(compile_fail_${FILENAME} 
      STATIC
      EXCLUDE_FROM_ALL
      ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.cpp
      )

    set_target_properties(compile_fail_${FILENAME}
      PROPERTIES
      ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      )

    set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.cpp}
      PROPERTIES
      OBJECT_OUTPUTS THIS_DOES_NOT_EXIST_TO_PROVOKE_REBUILD_EVERY_TIME
      )
    
    add_test(NAME compile_fail_${FILENAME}
      COMMAND
      ${CMAKE_CURRENT_SOURCE_DIR}/fail.sh 
      ${CMAKE_MAKE_PROGRAM} compile_fail_${FILENAME})

  endmacro()


  macro(add_compile_pass_test FILENAME)
    add_library(compile_pass_${FILENAME} 
      STATIC
      EXCLUDE_FROM_ALL
      ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.cpp
      )

    set_target_properties(compile_pass_${FILENAME}
      PROPERTIES
      ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      )

    set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.cpp}
      PROPERTIES
      OBJECT_OUTPUTS THIS_DOES_NOT_EXIST_TO_PROVOKE_REBUILD_EVERY_TIME
      )
    
    add_test(NAME compile_pass_${FILENAME}
      COMMAND
      ${CMAKE_CURRENT_SOURCE_DIR}/pass.sh 
      ${CMAKE_MAKE_PROGRAM} compile_pass_${FILENAME})

  endmacro()

  add_compile_fail_test(redeclare)
  #TODO need to add expected error here?
  add_compile_pass_test(tendril_static_time_correct)
  add_compile_pass_test(tendril_static_time_correct2)
  add_compile_fail_test(tendril_static_time_nonspore)

  # expect "module_must_be_named_compile_fail_wrong_cell_module_namex"
  add_compile_fail_test(wrong_cell_module_name)
  # expect "module_must_be_named_whatshouldinameyou"
  add_compile_fail_test(wrong_module_name)

  #  For each public header file, create a test that compilation is
  #  successful if this header file is the only thing included.
  #
  file(GLOB_RECURSE ECTO_HEADERS 
    RELATIVE ${ecto_SOURCE_DIR}/include 
    ${ecto_SOURCE_DIR}/include/ecto/*.hpp)

  foreach(HEADER ${ECTO_HEADERS})

    string(REPLACE "/" "_" sanitized ${HEADER})
    string(REPLACE "." "_" sanitized ${sanitized})

    set(testname header_compiles_standalone_${sanitized})
    set(testsrc ${CMAKE_CURRENT_BINARY_DIR}/${testname}.cpp)

    configure_file(header_compile_test.cpp.in
      ${testsrc}
      @ONLY)

    add_library(${testname}
      STATIC
      EXCLUDE_FROM_ALL
      ${testsrc}
      )

    set_source_files_properties(${testsrc}
      PROPERTIES
      OBJECT_OUTPUTS THIS_DOES_NOT_EXIST_TO_PROVOKE_REBUILD_EVERY_TIME
      )
    
    set_target_properties(${testname}
      PROPERTIES
      ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      )

    add_test(NAME ${testname}
      COMMAND
      ${CMAKE_MAKE_PROGRAM} ${testname}
      )

  endforeach()


  file(GLOB_RECURSE ECTO_HEADERS_ABSPATH 
    ${ecto_SOURCE_DIR}/include/ecto/*.hpp
    ${ecto_SOURCE_DIR}/src/**/*.hpp)

  file(GLOB_RECURSE ECTO_SRC_ABSPATH 
    ${ecto_SOURCE_DIR}/src/*.cpp
    ${ecto_SOURCE_DIR}/test/cpp/*.cpp)

  file(GLOB_RECURSE ECTO_CMAKELISTS_ABSPATH 
    ${ecto_SOURCE_DIR}/CMakeLists.txt
    ${ecto_SOURCE_DIR}/*.cmake)

  add_test(NAME license_in_headers 
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/check_new_bsd_license.py ${ECTO_HEADERS_ABSPATH}
    )

  add_test(NAME license_in_src
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/check_new_bsd_license.py ${ECTO_SRC_ABSPATH}
    )

  add_test(NAME license_in_cmake
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/check_new_bsd_license.py ${ECTO_CMAKELISTS_ABSPATH}
    )

  file(GLOB ECTO_PY_ABSPATH
    ${ecto_SOURCE_DIR}/python/ecto/*.py
    ${ecto_SOURCE_DIR}/python/ecto/sphinx/*.py
    ${ecto_SOURCE_DIR}/test/scripts/*.py
    ${ecto_SOURCE_DIR}/samples/*.py
    )

  add_test(NAME license_in_py
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/check_new_bsd_license.py ${ECTO_PY_ABSPATH}
    )

endif()