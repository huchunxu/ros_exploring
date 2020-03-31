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
#include <ecto/log.hpp>
#include <ecto/ecto.hpp>
#include <ecto/cell.hpp>

#include <boost/foreach.hpp>
#include <boost/python/raw_function.hpp>
#include <boost/python/iterator.hpp>
#include <boost/python/slice.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/register_ptr_to_python.hpp>

#include <ecto/python/std_map_indexing_suite.hpp>
#include <ecto/python/raw_constructor.hpp>
#include <ecto/python/repr.hpp>
#include <string>
namespace bp = boost::python;
#include "tendril_spec.hpp"
#include "converter.hpp"

namespace ecto
{
  void inspect_impl(ecto::cell_ptr m, const boost::python::tuple& args,
                    const boost::python::dict& kwargs);

  namespace py
  {

    struct cellwrap: cell, bp::wrapper<cell>
    {
      cellwrap():initialized_(false){}

      void dispatch_start()
      {
        ECTO_SCOPED_CALLPYTHON();
        if (bp::override start = this->get_override("start"))
          start();
      }

      void dispatch_stop()
      {
        ECTO_SCOPED_CALLPYTHON();
        if (bp::override stop = this->get_override("stop"))
          stop();
      }

      void dispatch_declare_params(tendrils& params)
      {
        ECTO_SCOPED_CALLPYTHON();
        if (bp::override init = this->get_override("declare_params"))
          init(boost::ref(params));
      }

      void dispatch_declare_io(const tendrils&params, tendrils& inputs, tendrils& outputs)
      {
        ECTO_SCOPED_CALLPYTHON();
        if (bp::override declare_io = this->get_override("declare_io"))
          declare_io(boost::ref(params), boost::ref(inputs), boost::ref(outputs));
      }

      void dispatch_configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        ECTO_SCOPED_CALLPYTHON();
        if (bp::override config = this->get_override("configure"))
          config(boost::ref(params), boost::ref(inputs), boost::ref(outputs));
      }

      void dispatch_activate()
      {
        ECTO_SCOPED_CALLPYTHON();
        if (bp::override activate = this->get_override("activate"))
          activate();
      }

      void dispatch_deactivate()
      {
        ECTO_SCOPED_CALLPYTHON();
        if (bp::override deactivate = this->get_override("deactivate"))
          deactivate();
      }

      struct YouveBeenServed
      {
        void operator()(const tendrils::value_type& t)
        {
          t.second->notify();
        }
      };

      ReturnCode dispatch_process(const tendrils& inputs, const tendrils& outputs)
      {
        ECTO_SCOPED_CALLPYTHON();
        int value = OK;
        std::for_each(inputs.begin(),inputs.end(), YouveBeenServed());
        if (bp::override proc = this->get_override("process"))
          {
            bp::object rval = proc(boost::ref(inputs), boost::ref(outputs));
            bp::extract<int> x(rval);
            if(x.check()){
              value = x();
            }
          }
        std::for_each(outputs.begin(),outputs.end(),YouveBeenServed());
        return ReturnCode(value);
      }

      bool init()
      {
        bool initialized = initialized_;
        initialized_ = false;
        return initialized;
      }

      std::string dispatch_name() const
      {
        bp::reference_existing_object::apply<cellwrap*>::type converter;
        PyObject* obj = converter(this);
        bp::object real_obj = bp::object(bp::handle<>(obj));
        bp::object n = real_obj.attr("__class__").attr("__name__");
        std::string nm = bp::extract<std::string>(n);
        return nm;
      }

      static std::string doc(cellwrap* mod)
      {
        bp::reference_existing_object::apply<cellwrap*>::type converter;
        PyObject* obj = converter(mod);
        bp::object real_obj = bp::object(bp::handle<>(obj));
        bp::object n = real_obj.attr("__class__").attr("__doc__");
        bp::extract<std::string> get_str(n);
        if (get_str.check())
          return get_str();
        return "No Doc str.";
      }

      cell_ptr dispatch_clone() const
      {
        throw std::logic_error("Clone is not implemented!");
        return cell_ptr();
      }
      bool initialized_;
    };

    const tendrils& inputs(cell& mod)
    {
      return mod.inputs;
    }
    tendrils& outputs(cell& mod)
    {
      return mod.outputs;
    }
    tendrils& params(cell& mod)
    {
      return mod.parameters;
    }

    void wrapModule()
    {
      //use private names so that python people know these are internal
      bp::class_<cell, boost::shared_ptr<cell>, boost::noncopyable>("_cell_cpp", bp::no_init)
        .def("type", &cell::type)
        .def("configure", ((void(cell::*)()) &cell::configure))
        .def("activate", ((void(cell::*)()) &cell::activate))
        .def("deactivate", ((void(cell::*)()) &cell::deactivate))
        ;
      BP_REGISTER_SHARED_PTR_TO_PYTHON(cell);

      bp::class_<cellwrap, boost::shared_ptr<cellwrap>, boost::noncopyable> ("_cell_base" /*bp::no_init*/)
        .def("_set_process_connected_inputs_only", &cell::set_process_connected_inputs_only)
        .def("_set_strand", &cell::set_strand)
        .def("_reset_strand", &cell::reset_strand)
        .def("construct", &inspect_impl)
        .def("declare_params", &cell::declare_params)
        .def("declare_io", ((void(cell::*)()) &cell::declare_io))
        .def("configure", ((void(cell::*)()) &cell::configure))
        .def("activate", ((void(cell::*)()) &cell::activate))
        .def("deactivate", ((void(cell::*)()) &cell::deactivate))
        .def("process_with_only_these_inputs", (void(cell::*)()) &cell::process_with_only_these_inputs)
        .def("process", (void(cell::*)()) &cell::process)
        .def("start", (void(cell::*)()) &cell::start)
        .def("stop", (void(cell::*)()) &cell::stop)
        .def("verify_params", &cell::verify_params)
        .def("verify_inputs", &cell::verify_inputs)

        .add_property("inputs", make_function(&inputs, bp::return_internal_reference<>()))
        .add_property("outputs", make_function(outputs, bp::return_internal_reference<>()))
        .add_property("params", make_function(params, bp::return_internal_reference<>()))
        .def("typename", &cell::type)
        .def("name",(((std::string(cell::*)() const) &cell::name)))
        .def("name",(((void(cell::*)(const std::string&)) &cell::name)))

        .def("doc", &cellwrap::doc)
        .def("short_doc",(std::string(cell::*)() const) &cell::short_doc)
        .def("gen_doc", &cell::gen_doc)
        .def("__getitem__", getitem_str)
        .def("__getitem__", getitem_tuple)
        .def("__getitem__", getitem_list)
        .def("__getitem__", getitem_slice)
        ;
      BP_REGISTER_SHARED_PTR_TO_PYTHON(cellwrap);

      bp::def("__getitem_str__", getitem_str);
      bp::def("__getitem_slice__", getitem_slice);
      bp::def("__getitem_tuple__", getitem_tuple);
      bp::def("__getitem_list__", getitem_list);

      bp::class_<TendrilSpecification>("TendrilSpecification")
        .def_readwrite("module_input", &TendrilSpecification::mod_input)
        .def_readwrite("module_output", &TendrilSpecification::mod_output)
        .def_readwrite("key", &TendrilSpecification::key)
        .def("to_tendril",&TendrilSpecification::toTendril)
        ;

      bp::class_<TendrilSpecifications>("TendrilSpecifications", bp::init<bp::list>())
        .def("to_tendrils", &TendrilSpecifications::toTendrils)
        .staticmethod("to_tendrils")
        .def("to_spec", &TendrilSpecifications::toSpec)
        .def("__rshift__", rshift_spec)
        .def("__rshift__", rshift_spec_tuples)
        ;

      bp::enum_<tendril_type>("tendril_type")
        .value("INPUT",INPUT)
        .value("OUTPUT",OUTPUT)
        .value("PARAMETER",PARAMETER)
        .export_values()
        ;
      bp::enum_<ecto::ReturnCode>("ReturnCode")
        .value("OK",OK)
        .value("QUIT",QUIT)
        .value("DO_OVER",DO_OVER)
        .export_values()
        ;
    }
  }

  void inspect_impl(ecto::cell_ptr m, const boost::python::tuple& args,
                    const boost::python::dict& kwargs)
  {

    if (bp::len(args) > 1)
      throw std::runtime_error("Only one non-keyword argument allowed, this will specify instance name");

    if (bp::len(args) == 0)
      {
        // generate default name == type
        m->name(m->type());
      }
    else
      {
        bp::extract<std::string> e(args[0]);
        if (! e.check())
          throw std::runtime_error("Non-keyword argument (instance name) not convertible to string.");
        m->name(e());
      }
    m->declare_params();

    ECTO_LOG_DEBUG("inspect_impl %s", m->name());

    bp::list l = kwargs.items();
    for (int j = 0; j < bp::len(l); ++j)
      {
        bp::object key = l[j][0];
        bp::object value = l[j][1];
        std::string keystring = bp::extract<std::string>(key);
        if (keystring == "strand")
          {
            ecto::strand s = bp::extract<ecto::strand>(value);
            m->strand_ = s;
            ECTO_LOG_DEBUG("Found a strand for cell %s, id=%p", m->name() % s.id());
          }
        else if ( keystring == "connected_inputs_only")
        {
          bool result = bp::extract<bool>(value);
          m->set_process_connected_inputs_only(result);
        }
        else
          {
            tendril_ptr tp = m->parameters[keystring];
            try{
              *tp << value;
            }catch(ecto::except::TypeMismatch& e)
              {
                e << except::tendril_key(keystring);
                e << except::cell_name(m->name());
                throw;
              }
            tp->user_supplied(true);
            tp->dirty(true);
          }
      }
    m->declare_io();
  }
}

