#include <ecto/ecto.hpp>
#include <iostream>
#include <string>

namespace tutorial
{
  using ecto::tendrils;

  struct Printer03
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("prefix", "A prefix for printing");
    }
    
    static void 
    declare_io(const tendrils& /*params*/, tendrils& in, tendrils& /*out*/)
    {
      in.declare<std::string>("input", "A string to print", "Default Value");
    }
    
    void
    configure(const tendrils& params, const tendrils& /*in*/, const tendrils& /*out*/)
    {
      params["prefix"] >> prefix_;
    }
    
    int
    process(const tendrils& in, const tendrils& /*out*/)
    {
      std::cout << prefix_ << " >>> " << in.get<std::string>("input") << std::endl;
      return ecto::OK;
    }
    
    std::string prefix_;
  };
}

//register our cell with the existing ecto cell 'tutorial' that is declared in tutorial.cpp
ECTO_CELL(tutorial, tutorial::Printer03, "Printer03", "Prints a string to standard output.");
