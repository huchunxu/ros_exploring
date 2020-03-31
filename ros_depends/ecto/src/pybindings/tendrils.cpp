//
// Copyright (c) 2011, Willow Garage, Inc.
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
#include <ecto/all.hpp>

#include <ecto/python/std_map_indexing_suite.hpp>
#include <boost/foreach.hpp>
#include <ecto/serialization/registry.hpp>

namespace bp = boost::python;

namespace ecto
{
  namespace py
  {
    namespace
    {

      void declareTendril_no_default(tendrils& t, const std::string& name,
                      const std::string& doc)
      {
        t.declare<ecto::tendril::none> (name, doc);
      }

      void declareTendril(tendrils& t, const std::string& name,
                      const std::string& doc="Doc me!", bp::object o=bp::object())
      {
        t.declare<bp::object> (name, doc, o);
      }

      void declareTendrilPtr(tendrils& t, const std::string& name,
                      tendril_ptr x)
      {
        t.declare(name,x);
      }

      bp::list tendril_members(const tendrils& t)
      {
        bp::list l;
        for (tendrils::const_iterator iter = t.begin(), end = t.end(); iter != end; ++iter)
          l.append(iter->first);
        return l;
      }

      std::string strTendril(const tendrils& t)
      {
        std::string s = "tendrils:\n";
        for (tendrils::const_iterator iter = t.begin(), end = t.end(); iter != end; ++iter)
        {
          s += "    " + iter->first + " [" + iter->second->type_name() + "]\n";
        }
        return s;
      }

      bp::object tendril_get(const tendrils& ts, const std::string& name)
      {
        if (name == "__members__")
          return tendril_members(ts);
        if (name == "__objclass__")
          return bp::object();
        const tendril& t = *ts[name];
        bp::object o;
        t >> o;
        return o;
      }

      void tendril_set(tendrils& ts, const std::string& name, bp::object obj)
      {
        tendril_ptr t = ts[name];
        t << obj;
        t->dirty(true);
        t->user_supplied(true);
      }

      void tendrils_notify(tendrils& ts)
      {
        BOOST_FOREACH(tendrils::value_type& val, ts)
        {
          val.second->notify();
        }
      }

      tendril_ptr tendril_at(tendrils& ts, const std::string& name)
      {
        return ts[name];
      }

      void tendrils_save(const tendrils& ts, std::ostream& o)
      {
        boost::archive::binary_oarchive oa(o, boost::archive::no_header);
        oa << ts;
      }

      void tendrils_load(tendrils& ts, std::istream& i){
        boost::archive::binary_iarchive ia(i,  boost::archive::no_header);
        ia >> ts;
      }
    }

    void wrapTendrils()
    {
      using bp::arg;
      bp::class_<tendrils, boost::shared_ptr<tendrils>, boost::noncopyable>("Tendrils")
        .def(bp::std_map_indexing_suite<tendrils, false>())
        .def("declare",&declareTendril_no_default)
        .def("declare", &declareTendril)
        .def("declare", &declareTendrilPtr)
        .def("__str__", &strTendril)
        .def("__getattr__", &tendril_get)
        .def("__setattr__", &tendril_set)
        .def("__getitem__", &tendril_get)
        .def("at",tendril_at)
        .def("notify",tendrils_notify)
        .def("save", tendrils_save)
        .def("load", tendrils_load)
        .enable_pickling()
        ;
    }
  }
}

