//start
#include <ecto/ecto.hpp>
namespace overview
{
  struct NopCell
  {
  };
}
ECTO_CELL(ecto_overview, overview::NopCell, "NopCell",
          "A cell that can't do anything.");
//end
