#include <ecto/ecto.hpp>
#include <iostream>

using ecto::tendrils;

struct EnumAsInt
{
  enum Strategy {
    FAST, SLOW, ADAPTIVE
  };

  static void declare_params(tendrils& p)
  {
    p.declare<int>("strategy", "How to run our algorithm");
  }

  void configure(const tendrils& p, const tendrils& i, const tendrils& o)
  {
    int n = p.get<int>("strategy");
    switch (n) {
    case FAST:
      std::cout << "running FAST\n"; break;
    case SLOW:
      std::cout << "running SLOW\n"; break;
    case ADAPTIVE:
      std::cout << "running ADAPTIVE\n"; break;
    default:
      std::cout << "BAD!  Unhandled value of enum parameter";
    }

  }
};

ECTO_CELL(ecto_examples, EnumAsInt, "EnumAsInt", 
          "Example of how *not* to handle parameters that are enums");
