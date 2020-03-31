//
// Copyright (c) 2012, Industrial Perception, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <Python.h>
#include <boost/python.hpp>
// The next line needs to be here for Ubuntu Lucid
#include <boost/date_time/time_resolution_traits.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/date_time/posix_time/time_parsers.hpp>
#include <ecto/time.hpp>

namespace bp  = boost::python;
namespace bpt = boost::posix_time;

namespace ecto
{
namespace py
{
namespace {

template<typename T>
std::string to_str(const T & t)
{
  std::ostringstream oss;
  oss << t;
  return oss.str();
}

ptime_t * ptime_from_string(const std::string & s) {
  ptime_t * p = new ptime_t;
  *p = bpt::time_from_string(s);
  return p;
}

template<typename T>
bool ne(const T & lhs, const T & rhs)
{ return !(lhs == rhs); }

template<typename T>
bool le(const T & lhs, const T & rhs)
{ return lhs < rhs || lhs == rhs; }

template<typename T>
bool ge(const T & lhs, const T & rhs)
{ return lhs > rhs || lhs == rhs; }

template<typename T>
T add(const T & lhs, const T & rhs)
{ return lhs + rhs; }

// NOTE: Just implicitly convert to a duration to simplify operator defs.
ecto::ptime_duration_t hours(int i) { return bpt::hours(i); }
ecto::ptime_duration_t minutes(int i) { return bpt::minutes(i); }
ecto::ptime_duration_t seconds(int i) { return bpt::seconds(i); }
ecto::ptime_duration_t millisec(int i) { return bpt::millisec(i); }
ecto::ptime_duration_t microsec(int i) { return bpt::microsec(i); }
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
ecto::ptime_duration_t nanosec(int i) { return bpt::nanosec(i); }
#endif

} // End of anonymous namespace.

void wrap_ptime()
{
  bp::class_<ecto::ptime_duration_t>("ptime_duration")
    .def(bp::init<ecto::ptime_duration_t>())
    .def("__str__", &ecto::py::to_str<ecto::ptime_duration_t>)
    .def("__eq__", (bool (*)(const ecto::ptime_duration_t &, const ecto::ptime_duration_t &)) &bpt::posix_time_system::is_equal)
    .def("__ne__",  &ecto::py::ne<ecto::ptime_duration_t>)
    .def("__lt__",  (bool (*)(const ecto::ptime_duration_t &, const ecto::ptime_duration_t &))&bpt::posix_time_system::is_less)
    .def("__le__",  &ecto::py::le<ecto::ptime_duration_t>)
    .def("__ge__",  &ecto::py::ge<ecto::ptime_duration_t>)
    .def("__sub__", (ecto::ptime_duration_t(*)(const ecto::ptime_duration_t &, const ecto::ptime_duration_t &))&bpt::posix_time_system::subtract_times)
    .def("__add__", &ecto::py::add<ecto::ptime_duration_t>);

  bp::def("hours", & ecto::py::hours);
  bp::def("minutes", & ecto::py::minutes);
  bp::def("seconds", & ecto::py::seconds);
  bp::def("millisec", & ecto::py::millisec);
  bp::def("microsec", & ecto::py::microsec);
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
  bp::def("nanosec", & ecto::py::nanosec);
#endif

  bp::class_<ecto::ptime_t>("ptime")
    .def(bp::init<ecto::ptime_t>())
    .def("__init__", bp::make_constructor(&ptime_from_string))
    .def("__str__", &ecto::py::to_str<ecto::ptime_t>)
    .def("__eq__",  (bool (*)(const ecto::ptime_t &, const ecto::ptime_t &))&bpt::posix_time_system::is_equal)
    .def("__ne__",  &ecto::py::ne<ecto::ptime_t>)
    .def("__lt__",  (bool (*)(const ecto::ptime_t &, const ecto::ptime_t &))&bpt::posix_time_system::is_less)
    .def("__le__",  &ecto::py::le<ecto::ptime_t>)
    .def("__ge__",  &ecto::py::ge<ecto::ptime_t>)
    .def("__sub__", (ecto::ptime_duration_t(*)(const ecto::ptime_t &, const ecto::ptime_t &))&bpt::posix_time_system::subtract_times)
    .def("__add__", (ecto::ptime_t(*)(const ecto::ptime_t &, const ecto::ptime_duration_t &))&bpt::posix_time_system::add_time_duration);

  bp::def("microseconds",&ecto::microseconds);
  bp::def("local_time",&ecto::local_time);
  bp::def("universal_time",&ecto::universal_time);
  bp::def("ptime_from_string", &bpt::time_from_string);
  bp::def("ptime_from_iso_string", &bpt::from_iso_string);
}

} // End of namespace py.
} // End of namespace ecto.
