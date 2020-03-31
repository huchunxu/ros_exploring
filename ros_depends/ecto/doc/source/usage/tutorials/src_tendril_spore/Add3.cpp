#include <ecto/ecto.hpp>

using ecto::tendrils;
namespace tutorial
{
  struct Add
  {
    static void
    declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      in.declare(&Add::a_, "a", "1st operand.").required(true);
      in.declare(&Add::b_, "b", "2nd operand.").required(true);
      out.declare(&Add::output_, "output", "Result of a + b.");
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      *output_ = (*a_ + *b_);
      return ecto::OK;
    }
  };
  ecto::spore<double> output_, a_, b_;
}

ECTO_CELL(tutorial, tutorial::Add, "Add", "Adds two integers together.");
