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
#include <Python.h>
#include <boost/python.hpp>

#include <ecto/tendril.hpp>

#include <boost/foreach.hpp>

#include <ecto/serialization/tendril.hpp>

#include "converter.hpp"

namespace bp = boost::python;

namespace ecto
{
namespace py
{

tendril_ptr tendril_ctr()
{
  return boost::shared_ptr<tendril>(new tendril(bp::object(),"A pythonic tendril."));
}

tendril_ptr tendril_ctr1(const bp::object & object)
{
  //allow the tendril to have no type...
#ifdef BOOST_PYTHON_OBJECT_HAS_IS_NONE
  if(object.is_none()){
#else
  if(object.ptr() == Py_None){
#endif
    return boost::shared_ptr<tendril>(new tendril);
  }
  return boost::shared_ptr<tendril>(new tendril(object,"A pythonic tendril."));
}

// this is probably a terrible thing to do, but it is needed for pickle
// an alternative is to override __new__
tendril_ptr tendril_ctr2(std::string s, std::string useless)
{
  return boost::shared_ptr<tendril>(new tendril(registry::tendril::get(s)));
}

std::string tendril_type_name(tendril_ptr t)
{
  return t->type_name();
}

std::string tendril_doc(tendril_ptr t)
{
  return t->doc();
}

void tendril_set_doc(tendril_ptr t, const std::string& doc)
{
  return t->set_doc(doc);
}

bp::object tendril_get_val(tendril_ptr t)
{
  bp::object o;
  t >> o;
  return o;
}

void tendril_set_val(tendril_ptr t, bp::object val)
{
  t << val;
  t->dirty(true);
  t->user_supplied(true);
}
void tendril_copy_val(tendril_ptr t, tendril_ptr tv)
{
  t << *tv;
}
bool tendril_user_supplied(tendril_ptr t)
{
  return t->user_supplied();
}

bool tendril_has_default(tendril_ptr t)
{
  return t->has_default();
}

bool tendril_dirty(tendril_ptr t)
{
  return t->dirty();
}

bool tendril_required(tendril_ptr t)
{
  return t->required();
}

bp::object py_tendril_reg_list(){
  bp::list l;
  BOOST_FOREACH(const std::string& x, ecto::registry::tendril::type_names())
      l.append(bp::str(x));
  return l;
}
namespace io = boost::iostreams;

std::string py_tendril_save(const tendril& t)
{
  //todo allow python to pass in a buffer so copies don't need to be made.
  std::string buf;
  ecto::serialization::save(buf, t);
  return buf;
}

void py_tendril_load(tendril& t,const std::string& str){
  ecto::serialization::load(str, t);
}

void wrapConnection(){
  bp::class_<tendril,boost::shared_ptr<tendril> > Tendril_("Tendril",
      "The Tendril is the slendor winding organ of ecto.\n"
      "It is a type erasing holder with meta data that enable introspection.");
    Tendril_.def("__init__", bp::make_constructor(tendril_ctr));
    Tendril_.def("__init__", bp::make_constructor(tendril_ctr1));
    Tendril_.def("__init__", bp::make_constructor(tendril_ctr2));
    Tendril_.add_property("doc",tendril_doc,&tendril::set_doc, "A doc string that describes the purpose of this tendril.");
    Tendril_.add_property("type_name",tendril_type_name, "The type of the value held by the tendril." );
    Tendril_.add_property("val", tendril_get_val,tendril_set_val, "The value held by the tendril.\n"
        "It requires boost::python bindings to be accessible from python.\n"
        "If none are available it will be None.");
    Tendril_.add_property("user_supplied",tendril_user_supplied, "Has the value been set by the user?");
    Tendril_.add_property("has_default",tendril_has_default, "Does the tendril have an explicit default value?\n"
        "Remember that the implicit default is always the default constructed type.");
    Tendril_.add_property("required",tendril_required, "Is this tendril required to be connected?");
    Tendril_.add_property("dirty",tendril_dirty, "Has the tendril changed since the last time?");
    Tendril_.def("get",tendril_get_val, "Gets the python value of the object.\n"
    "May be None if python bindings for the type held do not have boost::python bindings available from the current scope."
    );
    Tendril_.def("set",tendril_set_val, "Assuming the value held by the tendril has boost::python bindings,\nthis will copy the value of the given python object into the value held by the tendril.");
    Tendril_.def("copy_value",tendril_copy_val, "Copy from one tendril to the other.");
    Tendril_.def("notify",&tendril::notify, "Force updates.");
    Tendril_.def("save",&py_tendril_save);
    Tendril_.def("load",&py_tendril_load);
    Tendril_.def("createT",&registry::tendril::get, "Create a tendril of the c++ type.", bp::return_value_policy<bp::return_by_value>());
    def("make_tendril",&registry::tendril::get, "Create a tendril of the c++ type.", bp::return_value_policy<bp::return_by_value>());
    Tendril_.staticmethod("createT");
    Tendril_.def("listT", &py_tendril_reg_list);
    Tendril_.staticmethod("listT");
    Tendril_.enable_pickling();
  BP_REGISTER_SHARED_PTR_TO_PYTHON(tendril);

}
}
}


