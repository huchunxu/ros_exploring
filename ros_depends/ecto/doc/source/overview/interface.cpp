//start
#include <ecto/ecto.hpp>
using ecto::tendrils;
namespace overview
{
  struct InterfaceCell
  {
    static void
    declare_params(tendrils& params);

    static void
    declare_io(const tendrils& params, tendrils& in, tendrils& out);

    void
    configure(const tendrils& params, const tendrils& in, const tendrils& out);

    int
    process(const tendrils& in, const tendrils& out);
  };
}
ECTO_CELL(ecto_overview, overview::InterfaceCell, "InterfaceCell",
          "A cell cell implementing the entire ecto interface");
//end

//impl_start
namespace overview
{
  void
  InterfaceCell::declare_params(tendrils & p)
  {
    //...
  }

  void
  InterfaceCell::declare_io(const tendrils & p, tendrils & i, tendrils & o)
  {
    //...
  }

  void
  InterfaceCell::configure(const tendrils & p, const tendrils & i, const tendrils & o)
  {
    //....
  }

  int
  InterfaceCell::process(const tendrils & i, const tendrils & o)
  {
    return ecto::OK;
  }
}
//impl_end
