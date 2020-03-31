#include <ecto/ecto.hpp>
#include <iostream>

namespace tutorial
{
  using ecto::tendrils;
  struct Hello
  {
    /* We only a process function for now, and it just prints somethin
     * Its inputs and outputs are ignored in that case
     */
    int
    process(const tendrils& /*in*/, const tendrils& /*out*/)
    {
      std::cout << "Hello" << std::endl;
      return ecto::OK;
    }
  };
}

// This macro is required to register the cell with the module
// first argument: the module that was defined in the tutorial.cpp
// second argument: the cell we want to expose in the module
// third argument: the name of that cell as seen in Python
// fourht argument: a description of what that cell does
ECTO_CELL(tutorial, tutorial::Hello, "Hello", "Prints 'Hello' to standard output.");
