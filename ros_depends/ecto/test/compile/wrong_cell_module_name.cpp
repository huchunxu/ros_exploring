#include <ecto/ecto.hpp>

struct SomeCell
{

};

ECTO_CELL(compile_fail_wrong_cell_module_namex, SomeCell,"SomeCell","Doesn't do much here.");

