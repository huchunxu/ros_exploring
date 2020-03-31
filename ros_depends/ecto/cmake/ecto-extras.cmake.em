# ==============================================================================
#  The ecto CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    find_package(ecto REQUIRED)
#    ectomodule(mymodule src/module.cpp)
#    link_ecto(mymodule ${MY_LIB_DEPS})
#
#    This file defines the following macros:
#	   ectomodule(module_name ${srcs}) #This creates a single ecto module
#                                      #shared library loadable from python,
#                                      #will add includes, links to bare necessities and mangles the target name with ${name}_ectomodule
#	   link_ecto(module_name ${libs})  #Link to additional libraries
# ==============================================================================
get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(${SELF_DIR}/ectoMacros.cmake)
include(${SELF_DIR}/doc.cmake)
include(${SELF_DIR}/git.cmake)

# ============== dependencies ==================================================
find_package(Threads)
find_package(Boost COMPONENTS
  python
  thread
  system
  REQUIRED
  )
set(Python_ADDITIONAL_VERSIONS 2.7)
find_package(PythonLibs REQUIRED)


include(${SELF_DIR}/ectoMacros.cmake)

@[if DEVELSPACE]@
include_directories(@(CATKIN_DEVEL_PREFIX)/include)
@[end if]@
