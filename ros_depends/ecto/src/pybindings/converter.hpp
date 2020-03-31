
#ifndef __BP_REGISTER_SHARED_PTR_CONVERTER_HPP__
#define __BP_REGISTER_SHARED_PTR_CONVERTER_HPP__
#include <boost/python/register_ptr_to_python.hpp>

namespace bp = boost::python;

/* Fix to avoid registration warnings in pycaffe (#3960) */
#define BP_REGISTER_SHARED_PTR_TO_PYTHON(PTR) do { \
  const boost::python::type_info info = \
    boost::python::type_id<boost::shared_ptr<PTR > >(); \
  const boost::python::converter::registration* reg = \
    boost::python::converter::registry::query(info); \
  if (reg == NULL) { \
    bp::register_ptr_to_python<boost::shared_ptr<PTR > >(); \
  } else if ((*reg).m_to_python == NULL) { \
    bp::register_ptr_to_python<boost::shared_ptr<PTR > >(); \
  } \
} while (0)

#endif
