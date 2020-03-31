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
#include <ecto/ecto.hpp>
#include <boost/python/errors.hpp>
#undef BOOST_EXCEPTION_DYNAMIC_TYPEID
#define BOOST_EXCEPTION_DYNAMIC_TYPEID(x) name_of(x)

namespace bp = boost::python;
using namespace ecto::except;
namespace ecto
{

  namespace py
  {
    PyObject* ectoexception;

    template<typename ExceptionType>
    struct Translate_
    {
      static void
      translate(const ExceptionType & x)
      {
        std::string di = except::diagnostic_string(x);
        PyErr_SetString(Exc_Type_, di.c_str());
      }
      static PyObject* Exc_Type_;
    };

    template<typename ExceptionType>
    PyObject* Translate_<ExceptionType>::Exc_Type_;

    template<typename ExceptionType>
    void register_exception(const char* name, const char* fullname)
    {
      PyObject* newex = PyErr_NewException(const_cast<char*>(fullname),
                                           ectoexception, /*dict*/NULL);
      Py_INCREF(newex);
      PyModule_AddObject(bp::scope().ptr(), const_cast<char*>(name), newex);

      Translate_<ExceptionType>::Exc_Type_ = newex;
      bp::register_exception_translator<ExceptionType>(&Translate_<ExceptionType>
                                                       ::translate);
    }

    void
    wrap_except()
    {
      ectoexception = PyErr_NewException(const_cast<char*>("ecto.EctoException"),
                                         PyExc_RuntimeError, NULL);
      Py_INCREF(ectoexception);
      PyModule_AddObject(bp::scope().ptr(), "EctoException", ectoexception);

      Translate_<EctoException>::Exc_Type_ = ectoexception;
      bp::register_exception_translator<EctoException>
        (&Translate_<EctoException>::translate)
        ;


#define REGISTER_EXCEPTION(r, data, E) \
      register_exception<E>(BOOST_PP_STRINGIZE(E), "ecto." BOOST_PP_STRINGIZE(E));
      BOOST_PP_SEQ_FOR_EACH(REGISTER_EXCEPTION, ~, ECTO_EXCEPTIONS);

    }
  }
}

