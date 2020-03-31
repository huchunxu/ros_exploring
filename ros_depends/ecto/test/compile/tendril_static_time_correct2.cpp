#include <ecto/ecto.hpp>

using ecto::tendrils;
using ecto::spore;

struct StaticCell
{
  static void declare_io(const tendrils& p, tendrils& i, tendrils& o)
  {
    i.declare(&StaticCell::d,"d","doc",0.0);
  }
  spore<int> d;
};

