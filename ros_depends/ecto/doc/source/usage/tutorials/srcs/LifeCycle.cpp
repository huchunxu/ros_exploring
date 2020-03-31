#include <ecto/ecto.hpp>
#include <iostream>

using ecto::tendrils;
namespace lifecycle
{
  struct LifeCycle
  {
    static void
    declare_params(tendrils&)
    {
      std::cout << "static " << ecto::name_of<LifeCycle>() << "::declare_params" << std::endl;
    }

    static void
    declare_io(const tendrils&, tendrils&, tendrils&)
    {
      std::cout << "static " << ecto::name_of<LifeCycle>() << "::declare_io" << std::endl;

    }

    LifeCycle()
    {
      std::cout << ecto::name_of<LifeCycle>() << "::LifeCycle()  this=" << this << std::endl;
    }

    ~LifeCycle()
    {
      std::cout << ecto::name_of<LifeCycle>() << "::~LifeCycle()  this=" << this << std::endl;
    }

    void
    configure(const tendrils&, const tendrils&, const tendrils&)
    {
      std::cout << ecto::name_of<LifeCycle>() << "::configure  this=" << this << std::endl;
    }

    int
    process(const tendrils&, const tendrils&)
    {
      std::cout << ecto::name_of<LifeCycle>() << "::process  this=" << this << std::endl;
      return ecto::OK;
    }
  };
}

ECTO_DEFINE_MODULE(ecto_lifecycle)
{ }

ECTO_CELL(ecto_lifecycle, lifecycle::LifeCycle, "LifeCycle", "Chirps on each stage of life.");
