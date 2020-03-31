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
#include <ecto/python/streambuf.hpp>
namespace ecto
{
  namespace py
  {

    void
    wrap_python_streambuf()
    {
      using ecto::py::streambuf;
      using namespace boost::python;
      class_<streambuf, boost::shared_ptr<streambuf>, boost::noncopyable> sb("streambuf", no_init);
      sb.def(init<object&, std::size_t>((arg("file"), arg("buffer_size") = 0)));
      sb.def_readwrite("default_buffer_size", streambuf::default_buffer_size, "The default size of the buffer sitting "
                       "between a Python file object and a C++ stream.");
      using ecto::py::ostream;
      class_<std::ostream, boost::shared_ptr<std::ostream>, boost::noncopyable>("std_ostream", no_init);
      class_<ostream, boost::noncopyable, bases<std::ostream> > os("ostream", no_init);
      os.def(init<object&, std::size_t>((arg("python_file_obj"), arg("buffer_size") = 0)));
      os.def_readwrite("file",&ostream::get_original_file);

      using ecto::py::istream;
      class_<std::istream, boost::shared_ptr<std::istream>, boost::noncopyable>("std_istream", no_init);
      class_<istream, boost::noncopyable, bases<std::istream> > is("istream", no_init);
      is.def(init<object&, std::size_t>((arg("python_file_obj"), arg("buffer_size") = 0)));
      is.def_readwrite("file",&ostream::get_original_file);
    }

  }
}

