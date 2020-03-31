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
#include <ecto/tendrils.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/function.hpp>
#include <map>
#include <iostream>
namespace ecto
{
  namespace
  {
    template<typename T>
    static void
    print(std::ostream& out, const tendril& x)
    {
      out << x.get<T>();
    }

    template<>
    void
    print<boost::python::api::object>(std::ostream& out, const tendril& x)
    {
      namespace bp = boost::python;
      bp::object o;
      x >> o;
      out << std::string( bp::extract<std::string>(bp::str(o)));
    }
  }
  struct PrintFunctions
  {

    typedef boost::function<void(std::ostream& out, const tendril& x)> function_t;
    typedef std::map<const char*,function_t> ProcMap;
    ProcMap processes;
    PrintFunctions()
    {
      processes[ecto::name_of<int>().c_str()] = function_t(print<int>);
      processes[ecto::name_of<float>().c_str()] = function_t(print<float>);
      processes[ecto::name_of<double>().c_str()] = function_t(print<double>);
      processes[ecto::name_of<bool>().c_str()] = function_t(print<bool>);
      processes[ecto::name_of<std::string>().c_str()] = function_t(print<std::string>);
      processes[ecto::name_of<boost::python::api::object>().c_str()] = function_t(print<boost::python::api::object>);
    }

    void
    print_tendril(std::ostream& out, const tendril& t) const
    {
      ProcMap::const_iterator it = processes.find(t.type_id());
      if (it != processes.end())
      {
        it->second(out, t);
      }
      else
      {
        out << t.type_name() << "(?)";
      }
    }
  };

  const PrintFunctions pf;
  struct print_tendril_simple
  {
    print_tendril_simple(std::ostream& ss)
      : ss(ss)
    { }
    void
    operator()(const std::pair<std::string, tendril_ptr>& tp)
    {
      ss << " '" << tp.first << "':type(" << tp.second->type_name() << ")";
    }
    std::ostream& ss;
  };

  struct print_tendril
  {
    print_tendril(std::ostream& ss)
      : ss(ss)
    { }

    void
    operator()(const std::pair<std::string, tendril_ptr>& tp)
    {
      //TODO
      //EAR: Seems like we would like to have the ability to customize the
      //  str representation and type_name for any tendril?
      //  Also these seem like they belong in the tendril itself.
      //  This could go into a type registry system. All the more reason for us to have one.

      std::stringstream tss;
      pf.print_tendril(tss, *tp.second);
      //default value

      ss << " - " << tp.first << " [" << tp.second->type_name() << "]";
      ss << (tp.second->has_default() ? (" default = " + tss.str()) : "");
      ss << (tp.second->required() ? " REQUIRED " : "");
      ss << "\n";

      std::string docstr = tp.second->doc();
      std::vector<std::string> doc_lines;
      std::string doc_str = tp.second->doc();
      boost::split(doc_lines, doc_str, boost::is_any_of(std::string("\n")));//get rid of warning on earlier versions of boost std::string("\n")
      for (size_t i = 0; i < doc_lines.size(); ++i)
        ss << "    " << doc_lines[i] << "\n";
      ss << "\n";
    }
    std::ostream& ss;
  };

  //////////////////////////////////////////////////////////////////////////////

  tendrils::tendrils() { }

  void
  tendrils::print_doc(std::ostream& out, const std::string& tendrils_name) const
  {
    if (storage.empty())
      return;
    out << tendrils_name << ":\n";
    // out << "---------------------------------\n\n";
    std::for_each(storage.begin(), storage.end(), print_tendril(out));
  }

  void
  tendrils::doesnt_exist(const std::string& name) const
  {
    std::stringstream ss;
    std::for_each(begin(),end(),print_tendril_simple(ss));
    BOOST_THROW_EXCEPTION(except::NonExistant()
                          << except::tendril_key(name)
                          << except::actualkeys_hint(ss.str()));
  }

  const tendril_ptr&
  tendrils::operator[](const std::string& name) const
  {
    storage_type::const_iterator it = storage.find(name);
    if (it == end())
      doesnt_exist(name);
    return it->second;
  }

  tendril_ptr&
  tendrils::operator[](const std::string& name)
  {
    storage_type::iterator it = storage.find(name);
    if (it == end())
      doesnt_exist(name);
    return it->second;
  }

  tendril_ptr
  tendrils::declare(const std::string& name, tendril_ptr t)
  {
    storage_type::iterator it = find(name);
    //if there are no exiting tendrils by the given name,
    //just add it.
    if (it == end())
    {
      storage.insert(std::make_pair(name, t));
    }
    else // we want to just return the existing tendril (so that
         // modules preconnected don't get messed up)...
    {
        BOOST_THROW_EXCEPTION(except::TendrilRedeclaration()
                              << except::tendril_key(name)
                              << except::prev_typename(it->second->type_name())
                              << except::cur_typename(t->type_name()));
          ;
    }
    return storage.at(name);
  }
}

