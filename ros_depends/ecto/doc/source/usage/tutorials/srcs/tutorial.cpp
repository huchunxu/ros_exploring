#include <ecto/ecto.hpp>

//Defines a top level ecto python cell with the name 'tutorial'
//Note that the shared library must be called tutorial.so
//In CMake this is achieved using the ecto specific macro:
//ectomodule(tutorial INSTALL
//                    DESTINATION whatever_folder
//                    tutorial.cpp
//                    <source1.cpp>
//                    <source2.cpp>
//)

ECTO_DEFINE_MODULE(tutorial) { }
