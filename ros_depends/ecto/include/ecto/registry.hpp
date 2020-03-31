/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#include <vector>
#include <boost/noncopyable.hpp>
#include <boost/function/function0.hpp>
#include <boost/make_shared.hpp>
#include <ecto/forward.hpp>
#include <ecto/util.hpp>
#include <ecto/cell.hpp>

namespace ecto {

  namespace py {
    void postregistration(const std::string&, const std::string&, const std::string&);
  }

  struct cell;

  namespace registry {

    typedef boost::shared_ptr<cell>(*factory_fn_t)();
    typedef void (*declare_params_t)(ecto::tendrils&);
    typedef void (*declare_io_t)(const ecto::tendrils&, ecto::tendrils&, 
                                 ecto::tendrils&);

    struct entry_t {
      factory_fn_t construct;
      declare_params_t declare_params;
      declare_io_t declare_io;

      boost::shared_ptr<cell> construct_() { return construct(); }
      void declare_params_(ecto::tendrils& t) { declare_params(t); }
      void declare_io_(const ecto::tendrils& p, ecto::tendrils& i, ecto::tendrils& o) 
      { 
        declare_io(p, i, o); 
      }
    };

    template <typename ModuleTag>
    struct module_registry : boost::noncopyable
    {
      typedef boost::function0<void> nullary_fn_t;

      void add(nullary_fn_t f)
      {
        regvec.push_back(f);
      }

      void go()
      {
        for (unsigned j=0; j<regvec.size(); ++j)
          regvec[j]();
      }

      std::vector<nullary_fn_t> regvec;

      static module_registry& instance()
      {
        static module_registry instance_;
        return instance_;
      }

    private:

      module_registry() { }
    };

    template <typename Module, typename T>
    struct registrator 
    {
      const char* name_;
      const char* docstring_;

      typedef ::ecto::cell_<T> cell_t;

      static boost::shared_ptr<cell> create()
      {
        return boost::shared_ptr<cell>(new cell_t);
      }

      explicit registrator(const char* name, const char* docstring) 
        : name_(name), docstring_(docstring) 
      { 
        // this fires the construction of proper python classes at import time
        module_registry<Module>::instance().add(boost::ref(*this));

        // this registers the functions needed to do the construction above
        entry_t e;
        e.construct = &create;// ecto::create_cell<T>;
        e.declare_params = (void (*)(tendrils&)) &cell_t::declare_params;
        e.declare_io = (void (*)(const tendrils&, tendrils&, tendrils&)) &cell_t::declare_io;
        register_factory_fn(name_of<T>(), e);
      }

      void operator()() const 
      {
        ecto::py::postregistration(name_, docstring_, name_of<T>());
      }
      const static registrator& inst;
    };
    
    entry_t lookup(const std::string& name);
    boost::shared_ptr<cell> create(const std::string& name);
    boost::shared_ptr<cell> create_initialized(const std::string& name);
    void register_factory_fn(const std::string& name, entry_t e);
  }
}

#define ECTO_MODULETAG(MODULE) namespace ecto { namespace tag { struct MODULE; } }
#define ECTO_CELL(MODULE, TYPE, NAME, DOCSTRING)                        \
  ECTO_ASSERT_MODULE_NAME(MODULE)                                       \
  ECTO_MODULETAG(MODULE)                                                \
  namespace ecto{ namespace registry {                                  \
    template<>                                                          \
    const ::ecto::registry::registrator< ::ecto::tag::MODULE,TYPE>&     \
    ::ecto::registry::registrator< ::ecto::tag::MODULE,TYPE>::inst      \
    (::ecto::registry::registrator< ::ecto::tag::MODULE,TYPE>(NAME, DOCSTRING)); \
  } }
  
#define ECTO_INSTANTIATE_REGISTRY(MODULE)                               \
  ECTO_MODULETAG(MODULE)                                                \
  template struct ::ecto::registry::module_registry< ::ecto::tag::MODULE>;

#define ECTO_REGISTER(MODULE)                                           \
    ::ecto::registry::module_registry< ::ecto::tag::MODULE>::instance().go();
