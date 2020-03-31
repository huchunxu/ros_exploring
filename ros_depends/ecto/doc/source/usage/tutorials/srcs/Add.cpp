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

    int
    process(const tendrils& in, const tendrils& out)
    {
      out.get<int>("output") = in.get<int>("a") + in.get<int>("b");
      return ecto::OK;
    }
  };
}

ECTO_CELL(tutorial, tutorial::Add, "Add", "Adds two integers together.");
