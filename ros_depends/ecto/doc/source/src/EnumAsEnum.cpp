#include <iostream>
#include <ecto/ecto.hpp>

using ecto::tendrils;

enum Strategy {
  FAST, SLOW, ADAPTIVE
};

struct EnumAsEnum
{
  static void declare_params(tendrils& p)
  {
    p.declare<Strategy>("strategy", "How to run our algorithm");
  }

  void configure(const tendrils& p, const tendrils& i, const tendrils& o)
  {
    Strategy s = p.get<Strategy>("strategy");
    switch (s) {
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

ECTO_CELL(ecto_examples, EnumAsEnum, "EnumAsEnum", 
          "This is how to defined enum parameters.  Like this.");
