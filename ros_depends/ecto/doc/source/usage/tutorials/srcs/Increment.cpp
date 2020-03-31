#include <ecto/ecto.hpp>

using ecto::tendrils;
namespace tutorial
{
  struct Increment
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<int>("start", "The starting value.", 0);
    }

    static void
    declare_io(const tendrils& params, tendrils& /*in*/, tendrils& out)
    {
      out.declare<int>("output", "An increasing integer", params.get<int>("start"));
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      //the get function of the tendrils returns a mutable reference.
      out.get<int>("output") += 1;
      return ecto::OK;
    }
  };
}

ECTO_CELL(tutorial, tutorial::Increment, "Increment", "Outputs a monotonically increasing integer.");
