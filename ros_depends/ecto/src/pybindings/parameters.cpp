#include <Python.h>
#include <boost/python.hpp>

#include <boost/python/class.hpp>
#include <boost/python/implicit.hpp>
#include <boost/python/module.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <ecto/parameters.hpp>
#include <ecto/util.hpp>

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
using namespace boost::python;

namespace ecto
{
  using boost::lexical_cast;
  using boost::str;
  using boost::format;

  template<typename T>
  struct py_bounded
  {
    typedef bounded<T> bounded_t;
    typedef boost::shared_ptr<bounded_t> bounded_ptr_t;

    static const std::string&
    name()
    {
      static const std::string name = "bounded_" + symbolic_name_of<T>();
      return name;
    }

    static bounded_ptr_t
    make_bounds(const T& value, const T& min, const T& max)
    {
      bounded_ptr_t x(new bounded_t(value, min, max));
      return x;
    }

    static T
    getValue(const bounded_t& x)
    {
      return x.value;
    }

    static void
    setValue(bounded_t& x, const T& value)
    {
      x = value;
    }

    static std::string
    repr(const bounded_t& x)
    {

      if (x.has_bounds)
        return str(format("%s(%s,%s,%s)") //
                   % name() //
                   % lexical_cast<std::string>(x.value) //
                   % lexical_cast<std::string>(x.min) //
                   % lexical_cast<std::string>(x.max));
      else
        return str(format("%s(%s)") //
                   % name() //
                   % lexical_cast<std::string>(x.value));
    }

    static void
    wrap()
    {
      class_<bounded_t, bounded_ptr_t> cls(name().c_str(), init<T>());
      //class
      cls.def("__init__", make_constructor(&py_bounded<T>::make_bounds)); //constructor that takes bounds.
      cls.def_readwrite("has_bounds", &bounded_t::has_bounds) //has bounds
      .def_readwrite("min", &bounded_t::min) //min member
      .def_readwrite("max", &bounded_t::max) //max member
      .add_property("value", &py_bounded<T>::getValue, &py_bounded<T>::setValue) //value member
      .def("check", &bounded_t::check) //
//      .def("set", &bounded_t::operator=()) //
      .def("bounds", &bounded_t::bounds) //bounds function
      .def("__repr__", &py_bounded<T>::repr);

      implicitly_convertible<bounded_t, T>();
      implicitly_convertible<T, bounded_t>();
    }
  };

#define ECTO_PY_BINDINGS_BOUNDED(r, data, T) \
  py_bounded<T>::wrap();

  namespace py
  {
    void
    wrap_parameters()
    {
      BOOST_PP_SEQ_FOR_EACH(ECTO_PY_BINDINGS_BOUNDED, _, ECTO_COMMON_TYPES);
    }
  }
}
