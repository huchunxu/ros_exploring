#include <ecto/ecto.hpp>

using ecto::tendrils;
using ecto::spore;
namespace ecto_test
{
  struct Add
  {
    static void declare_io(const tendrils& p, tendrils& i, tendrils& o)
    {
      i.declare(&Add::left_,"left", "Left hand operand.", 0.0);
      i.declare(&Add::right_,"right","Right hand operand.", 0.0);
      o.declare(&Add::out_,"out","The result.");
    }
    int process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
    {
      *out_ = *left_ + *right_;
      return ecto::OK;
    }
    spore<double> out_, left_, right_;
  };
}

ECTO_CELL(ecto_test, ecto_test::Add, "Add", "Add two doubles together.");
