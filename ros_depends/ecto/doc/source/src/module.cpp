#include <ecto/ecto.hpp>
#include <ecto/python.hpp>

#include <boost/python.hpp>

enum Strategy {
  FAST, SLOW, ADAPTIVE
};

ECTO_DEFINE_MODULE(ecto_examples)
{
  boost::python::enum_<Strategy>("Strategy")
    .value("FAST", FAST)
    .value("SLOW", SLOW)
    .value("ADAPTIVE", ADAPTIVE)
    .export_values()
    ;
}

