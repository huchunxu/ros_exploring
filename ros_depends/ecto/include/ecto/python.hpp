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
//do not include this in ecto lib files, only in client modules

#pragma push_macro("_POSIX_C_SOURCE")
#pragma push_macro("_XOPEN_SOURCE")
#undef _POSIX_C_SOURCE
#undef _XOPEN_SOURCE
#include <Python.h>
#include <boost/python.hpp>
#include <boost/thread/thread.hpp>
#include <map>
//#pragma pop_macro("_POSIX_C_SOURCE")
//#pragma pop_macro("_XOPEN_C_SOURCE")

#include <boost/noncopyable.hpp>

namespace ecto {
  namespace py {

    struct gilstatus
    {
      const char* file;
      unsigned line;
      const char* what;
      gilstatus(const char* f, unsigned l, const char* w);
    };

    //
    //  used in the interpreter thread during blocking operations
    //  RIAA-style  Py_BEGIN_ALLOW_THREADS
    //
    class scoped_gil_release : boost::noncopyable
    {
      static std::map<boost::thread::id, PyThreadState*> thread_states;
      bool mine;
      gilstatus mystatus;
    public:
      scoped_gil_release(const char* file, unsigned line);
      ~scoped_gil_release();
    };

    //
    //  For non-python created threads
    //
    class scoped_call_back_to_python : boost::noncopyable
    {
      PyGILState_STATE gilstate;
      bool have;
      gilstatus mystatus;

    public:

      scoped_call_back_to_python(const char* file, unsigned line);
      ~scoped_call_back_to_python();
    };
  }
}

#define ECTO_SCOPED_GILRELEASE() ecto::py::scoped_gil_release gilrelease ## __LINE__(__FILE__, __LINE__)
#define ECTO_SCOPED_CALLPYTHON() ecto::py::scoped_call_back_to_python gilcall ## __LINE__(__FILE__, __LINE__)
