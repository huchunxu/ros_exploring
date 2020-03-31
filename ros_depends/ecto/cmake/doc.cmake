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

set(ECTO_DOCS_DEPLOY_DESTINATION "OFF" CACHE STRING
  "Deploy destination for docs, or OFF.  Will be passed to rsync; may contain user@ syntax for ssh"
  )

if (NOT TARGET sphinx-doc)
  add_custom_target(sphinx-doc)
endif()

if(ECTO_DOCS_DEPLOY_DESTINATION)
    if (NOT TARGET sphinx-deploy)
        add_custom_target(sphinx-deploy)
    endif()
endif()

macro(ecto_find_sphinx)
  find_program(SPHINX_BUILD sphinx-build)
  if(SPHINX_BUILD)
    set(REQUIRED_SPHINX_VERSION "1.0.7")
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sphinx;print sphinx.__version__"
      OUTPUT_VARIABLE SPHINX_VERSION
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )
    if("${SPHINX_VERSION}" VERSION_LESS ${REQUIRED_SPHINX_VERSION})
      MESSAGE(WARNING "You version of sphinx (http://sphinx.pocoo.org) is ${SPHINX_VERSION}, required ${REQUIRED_SPHINX_VERSION}")
      if (UNIX)
        MESSAGE(WARNING "You may be able to update with 'pip install -U sphinx'")
      endif()
    endif()
  endif()
endmacro()

##
# sphinx(<SOURCE_DIR> [PATH1 [ PATH2 [ PATH3 ]]])
# SOURCE_DIR -> Where the conf.py is
# PATH(s) -> paths to prepend to the PYTHONPATH for the execution of sphinx-build
#
set(SPHINX_LIST ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/sphinx_projects.list
  CACHE INTERNAL "File containing master sphinx projects list"
  )
file(REMOVE ${SPHINX_LIST})

macro(ecto_sphinx SOURCE_DIR)
  ecto_find_sphinx()
  if(SPHINX_BUILD)

    file(APPEND ${SPHINX_LIST} "${PROJECT_NAME}\n")

    add_custom_target(${PROJECT_NAME}-sphinx)
    add_custom_command(TARGET ${PROJECT_NAME}-sphinx
      COMMAND
      ${CATKIN_ENV} ${SPHINX_BUILD} -aE -b html
#      -c ${ecto_SPHINX_DIR}
      -D html_title=${PROJECT_NAME}
      -D project=${PROJECT_NAME}
      -D ecto_docs_dir="${CMAKE_BINARY_DIR}/doc/html"
      ${SOURCE_DIR} ${CMAKE_BINARY_DIR}/doc/html/${PROJECT_NAME}
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      )
    add_dependencies(sphinx-doc ${PROJECT_NAME}-sphinx)
    if (ECTO_DOCS_DEPLOY_DESTINATION)
      add_custom_target(${PROJECT_NAME}-sphinx-deploy)
      add_custom_command(TARGET ${PROJECT_NAME}-sphinx-deploy
        COMMAND rsync --perms --chmod=a+rX -va ${CMAKE_BINARY_DIR}/doc/html/${PROJECT_NAME}/ ${ECTO_DOCS_DEPLOY_DESTINATION}/${PROJECT_NAME})
      add_dependencies(sphinx-deploy ${PROJECT_NAME}-sphinx-deploy)
    endif()
  endif()
endmacro()
