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
#include <ecto/cell.hpp>
#include <cassert>
#include <ecto/util.hpp>
#include <ecto/except.hpp>
#include <boost/exception/all.hpp>
#include <boost/thread.hpp>

#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/seq/for_each.hpp>



/*
 * Catch all and pass on exception.
 */
#define CATCH_ALL()                                                     \
  catch (const boost::thread_interrupted&)                              \
    {                                                                   \
      ECTO_TRACE_EXCEPTION("const boost::thread_interrupted&");         \
      throw;                                                            \
    }                                                                   \
  catch (ecto::except::NonExistant& e)                                  \
    {                                                                   \
      ECTO_TRACE_EXCEPTION("const ecto::except::NonExistant&");         \
      const std::string* key =                                          \
        boost::get_error_info<except::tendril_key>(e);                  \
      assert(key && "NonExistant was thrown w/o stating what it is that doesn't exist."); \
      e << except::hint(auto_suggest(*key, *this))                      \
        << except::cell_name(name())                                    \
        << except::cell_type(type())                               \
        << except::function_name(__FUNCTION__)                          \
        ;                                                               \
      throw;                                                            \
    }                                                                   \
  catch (ecto::except::EctoException& e)                                \
    {                                                                   \
      ECTO_TRACE_EXCEPTION("const ecto::except::EctoException&");       \
      e << except::cell_name(name())                                    \
        << except::cell_type(type())                               \
        << except::function_name(__FUNCTION__)                          \
        ;                                                               \
      throw;                                                            \
    }                                                                   \
  catch (std::exception& e)                                             \
    {                                                                   \
      ECTO_TRACE_EXCEPTION("std::exception&");                          \
      BOOST_THROW_EXCEPTION(except::CellException()                     \
                            << except::type(name_of(typeid(e)))         \
                            << except::what(e.what())                   \
                            << except::cell_name(name())                \
                            << except::cell_type(type())           \
                            << except::function_name(__FUNCTION__))     \
        ;                                                               \
    }                                                                   \
  catch(const boost::python::error_already_set&)                        \
    {                                                                   \
      ECTO_TRACE_EXCEPTION("boost::python::error_already_set&");        \
      throw;                                                            \
    }                                                                   \
  catch(...)                                                            \
    {                                                                   \
      ECTO_TRACE_EXCEPTION("...");                                      \
      BOOST_THROW_EXCEPTION(except::CellException()                     \
                            << except::what("(unknown exception)")      \
                            << except::cell_name(name())                \
                            << except::cell_type(type())           \
                            << except::function_name(__FUNCTION__))     \
        ;                                                               \
    }

namespace ecto
{

  const std::string&
  ReturnCodeToStr(int rval)
  {
    switch(rval)
    {
#define RETURN_NAME(r, data, NAME)                                    \
case ecto::NAME: {static std::string x = BOOST_PP_STRINGIZE(ecto::NAME); return x;}
      BOOST_PP_SEQ_FOR_EACH(RETURN_NAME, ~, ECTO_RETURN_VALUES)
      default:{ static std::string r = "Unknown return value."; return r;}
    }
  }

  std::string
  auto_suggest(const std::string& key, const cell& m)
  {
    std::string p_type, i_type, o_type, msg;
    bool in_p = m.parameters.find(key) != m.parameters.end();
    if(in_p) p_type = m.parameters.find(key)->second->type_name();

    bool in_i = m.inputs.find(key) != m.inputs.end();
    if(in_i) i_type = m.inputs.find(key)->second->type_name();

    bool in_o = m.outputs.find(key) != m.outputs.end();
    if(in_o) o_type = m.outputs.find(key)->second->type_name();

    if (in_p || in_i || in_o)
      return "\n  Hint   : '" + key + "' does exist in " + (in_p ? "parameters (type == " +p_type +") " : "") + (in_i ? "inputs (type == " +i_type +") "  : "")
          + (in_o ? "outputs (type == " +o_type +")" : "");
    else
      return "  Hint   : '" + key + "' does not exist in module.";
  }

  void sample_siggy(cell& c, unsigned firing)
  {
    ECTO_LOG_DEBUG("sample_siggy(%s, %u)", c.name() % firing);
  }

  cell::cell()
    : configured_(false)
    , activated_(false)
    , process_connected_inputs_only_(false)
  {
  }

  cell::~cell() { }

  void
  cell::declare_params()
  {
    try
    {
      dispatch_declare_params(parameters);
    } CATCH_ALL()
  }

  void
  cell::declare_io()
  {
    try
    {
      dispatch_declare_io(parameters, inputs, outputs);
    } CATCH_ALL()
  }

  void
  cell::configure()
  {
    if (configured_)
      return;

    init();
    try
    {
      dispatch_configure(parameters, inputs, outputs);
      configured_ = true;
    } CATCH_ALL()
  }

  void
  cell::activate()
  {
    if (activated_)
      return;

    configure(); //configure first...

    try
    {
      dispatch_activate();
      activated_ = true;
    } CATCH_ALL()
  }

  void
  cell::deactivate()
  {
    if (! activated_)
      return;

    try
    {
      dispatch_deactivate();
      activated_ = false;
    } CATCH_ALL()
  }

  void
  cell::start()
  {
    ECTO_LOG_DEBUG("*** %s", "notified of start");
    dispatch_start();
  }

  void
  cell::stop()
  {
    ECTO_LOG_DEBUG("*** %s", "notified of stop");
    dispatch_stop();
  }

  ReturnCode
  cell::process_with_only_these_inputs(const tendrils& connected_inputs)
  {
    configure();
    //trigger all parameter change callbacks...
    tendrils::iterator begin = parameters.begin(), end = parameters.end();

    while (begin != end)
    {
      try
      {
        begin->second->notify();
      } catch (const std::exception& e)
      {
        ECTO_TRACE_EXCEPTION("const std::exception& outside of CATCH ALL");
        BOOST_THROW_EXCEPTION(except::CellException()
                              << except::type(name_of(typeid(e)))
                              << except::what(e.what())
                              << except::cell_name(name())
                              << except::function_name(__FUNCTION__)
                              << except::when("While triggering param change callbacks"))
          ;
      }
      ++begin;
    }
    try
    {
      try
      {
        const ReturnCode rc = dispatch_process(connected_inputs, outputs);
        return rc;
      } catch (const boost::thread_interrupted&) {
        ECTO_TRACE_EXCEPTION("const boost::thread_interrupted&, returning QUIT instead of rethrow");
        return ecto::QUIT;
      }
    } CATCH_ALL()
  }

  ReturnCode
  cell::process()
  {
    return process_with_only_these_inputs(inputs);
  }

  std::string
  cell::gen_doc(const std::string& doc) const
  {
    std::stringstream ss;
    ss << name() << " (ecto::module):\n";
    ss << "\n";
    ss << "\n" << doc << "\n\n";
    parameters.print_doc(ss, "Parameters");
    inputs.print_doc(ss, "Inputs");
    outputs.print_doc(ss, "Outputs");
    return ss.str();
  }

  void
  cell::verify_params() const
  {
    tendrils::const_iterator it = parameters.begin(), end(parameters.end());
    while (it != end)
    {
      if (it->second->required() && !it->second->user_supplied())
      {
        BOOST_THROW_EXCEPTION(except::ValueRequired()
                              << except::tendril_key(it->first));
      }
      ++it;
    }
  }

  void
  cell::verify_inputs() const
  {
    for (tendrils::const_iterator it = inputs.begin(), end(inputs.end());
         it != end; ++it)
    {
      if (it->second->required() && !it->second->user_supplied())
      {
        BOOST_THROW_EXCEPTION(except::NotConnected()
                              << except::tendril_key(it->first));
      }
    }
  }

  cell::ptr
  cell::clone() const
  {
    cell::ptr cloned = dispatch_clone();
    cloned->declare_params();
    //copy all of the parameters by value.
    tendrils::iterator it = cloned->parameters.begin(),
      end = cloned->parameters.end();
    tendrils::const_iterator oit = parameters.begin();
    while (it != end)
    {
      it->second << *oit->second;
      ++oit;
      ++it;
    }
    cloned->declare_io();
    return cloned;
  }

  void cell::reset_strand() {
    ECTO_LOG_DEBUG("reset_strand (%p)", 0);
    strand_.reset();
  }

  void cell::set_strand(ecto::strand s) {
    ECTO_LOG_DEBUG("set_strand id=%p", s.id());
    strand_ = s;
  }

} // End of namespace ecto.

