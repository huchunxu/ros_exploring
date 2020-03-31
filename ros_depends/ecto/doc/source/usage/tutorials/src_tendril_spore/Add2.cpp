#include <ecto/ecto.hpp>

using ecto::tendrils;
namespace tutorial
{
  struct Add
  {
    static void
    declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      in.declare<int>("a", "1st operand.").required(true);
      in.declare<int>("b", "2nd operand.").required(true);
      out.declare<int>("output", "Result of a + b.");
    }

    void configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      output_ = o["output"];
      a_ = i["a"];
      b_ = i["b"];
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
