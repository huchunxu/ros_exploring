#include <ecto/ecto.hpp>

using ecto::tendrils;
using ecto::spore;
struct SporeCell
{
  static void declare_io(const tendrils& p, tendrils& i, tendrils& o)
  {
    i.declare<double>("d");
    o.declare<double>("out");
  }
  void configure(const tendrils& p, const tendrils& i, const tendrils& o)
  {
    d = i["d"];
    out = o["out"];
    i.declare<double>("d","a new d.");
  }

  int process(const tendrils& i, const tendrils& o)
  {
    *out = *d;
    return ecto::OK;
  }
  spore<double> d,out;
};

