//start
#include <ecto/ecto.hpp>
#include <iostream>

using ecto::tendrils;

struct Example01 
{
  static void declare_params(tendrils& p)
  {
    p.declare<int>("value", "Some integer");
  }

  void configure(const tendrils& p, const tendrils& i, const tendrils& o)
  {
    int n = p.get<int>("value");
    std::cout << "Value of n is " << n << "\n";
  }
};

ECTO_CELL(ecto_examples, Example01, "Example01", "Example");
//end
