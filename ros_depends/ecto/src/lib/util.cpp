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
#include <vector>
#include <iostream>
#include <boost/unordered_map.hpp>
#include <ecto/util.hpp>
#include <ecto/except.hpp>
#if !defined(_WIN32)
#include <cxxabi.h>
#endif
#include <string>
#include <stdlib.h>
#include <cstring>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <boost/algorithm/string/replace.hpp>

#if defined(_WIN32)
// #define UNDNAME_COMPLETE                 (0x0000)
// #define UNDNAME_NO_LEADING_UNDERSCORES   (0x0001) /* Don't show __ in calling convention */
// #define UNDNAME_NO_MS_KEYWORDS           (0x0002) /* Don't show calling convention at all */
// #define UNDNAME_NO_FUNCTION_RETURNS      (0x0004) /* Don't show function/method return value */
// #define UNDNAME_NO_ALLOCATION_MODEL      (0x0008)
// #define UNDNAME_NO_ALLOCATION_LANGUAGE   (0x0010)
// #define UNDNAME_NO_MS_THISTYPE           (0x0020)
// #define UNDNAME_NO_CV_THISTYPE           (0x0040)
// #define UNDNAME_NO_THISTYPE              (0x0060)
// #define UNDNAME_NO_ACCESS_SPECIFIERS     (0x0080) /* Don't show access specifier (public/protected/private) */
// #define UNDNAME_NO_THROW_SIGNATURES      (0x0100)
// #define UNDNAME_NO_MEMBER_TYPE           (0x0200) /* Don't show static/virtual specifier */
// #define UNDNAME_NO_RETURN_UDT_MODEL      (0x0400)
// #define UNDNAME_32_BIT_DECODE            (0x0800)
// #define UNDNAME_NAME_ONLY                (0x1000) /* Only report the variable/method name */
// #define UNDNAME_NO_ARGUMENTS             (0x2000) /* Don't show method arguments */
// #define UNDNAME_NO_SPECIAL_SYMS          (0x4000)
// #define UNDNAME_NO_COMPLEX_TYPE          (0x8000)

// extern "C"
// char * _unDName(
// char * outputString,
// const char * name,
// int maxStringLength,
// void * (* pAlloc )(size_t),
// void (* pFree )(void *),
// unsigned short disableFlags);
// _unDName(0, pName, 0, malloc, free, UNDNAME_NO_ARGUMENTS | UNDNAME_32_BIT_DECODE);

namespace abi
{
  char* __cxa_demangle(const char * const pName,...)
  {
    char * name = new char[std::strlen(pName)];
    std::strcpy(name,pName);
    return name;
  }
}
#endif

namespace ecto
{
  using namespace ecto::except;

  class type_mapping
  {
  public:
    const std::string&
    lookup(const std::type_info& ti)
    {
      const char* mangled = ti.name();
      if (!mangled)
        {
          BOOST_THROW_EXCEPTION(EctoException()
                                << diag_msg("Could get a type name for your type! The world must be ending."));
        }
      return lookup(mangled);
    }

    const std::string&
    lookup(std::string mangled)
    {
      if (mangled == typeid(mangled).name())
        mangled = "std::string";

      dict_t::iterator iter = m.find(mangled);
      if (iter != m.end())
        return iter->second;

      std::string& rv = m[mangled];

      int status=0;
      char* demangled = abi::__cxa_demangle(mangled.c_str(), 0, 0, &status);
      if (status != 0)
        rv = mangled;
      else
        rv = demangled;
      free(demangled);
      return rv;
    }

    static type_mapping& instance() {
      static type_mapping m;
      return m;
    }

  private:
    type_mapping() { }
    
    typedef boost::unordered_map<std::string, std::string> dict_t;
    dict_t m;
  };

  const std::string& name_of(const std::type_info &ti)
  {
    return type_mapping::instance().lookup(ti);
  }
  const std::string& name_of(const std::string &s)
  {
    return type_mapping::instance().lookup(s);
  }

  template<>
  const std::string& name_of<std::string>()
  {
    static const std::string& name_cache = "std::string";
    return name_cache;
  }

  std::string
  symbolic_name_of(const std::string& t_name)
  {
    std::string result = t_name;
    boost::replace_all(result, " ", "_");
    boost::replace_all(result, "<", "_");
    boost::replace_all(result, ">", "_");
    boost::replace_all(result, "::", "_");
    return result;
  }

  template<>
  const std::string& symbolic_name_of<std::string>()
  {
    static const std::string name_cache = symbolic_name_of("std::string");
    return name_cache;
  }
}


