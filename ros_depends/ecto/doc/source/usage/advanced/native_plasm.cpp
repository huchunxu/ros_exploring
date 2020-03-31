#include <ecto/ecto.hpp>
#include <ecto/serialization/registry.hpp>
#include <ecto/serialization/cell.hpp>
#include <ecto/serialization/plasm.hpp>

#include <ecto/scheduler.hpp>

int
main()
{
  ecto::plasm::ptr p(new ecto::plasm());
  //load up our plasm
  {
    std::ifstream in("printy.plasm");
    p->load(in);
  }

  //lets make some graphviz
  {
    std::ofstream graphviz("printy.plasm.dot");
    p->viz(graphviz);
  }

  //iterate over all cells in the plasm.
  std::cout << "** cell listing" << std::endl;
  std::vector<ecto::cell::ptr> cells = p->cells();
  for (size_t i = 0; i < cells.size(); i++)
  {
    std::cout << cells[i]->name() << std::endl;
  }

  //use a scheduler...
  ecto::scheduler sched(p);
  return sched.execute();
}
