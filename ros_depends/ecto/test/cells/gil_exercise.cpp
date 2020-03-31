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
#include <ecto/except.hpp>
#include <ecto/python.hpp>
#include <ecto/rethrow.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>

using namespace ecto::except;
namespace bp = boost::python;

namespace {
  boost::asio::io_service serv;
  boost::thread_group tgroup;
}

void call_back_to_python(const bp::object& obj)
{
  ECTO_START();
  ECTO_SCOPED_CALLPYTHON();{
  ECTO_SCOPED_CALLPYTHON();
  // TRICKY: Grabbing a lock here is ill-advised... apparently there really are threads in that there interpreter
  std::string s = str(boost::format("thread_%p") % boost::this_thread::get_id());

  obj(s);
  }
  ECTO_FINISH();
}

void start_gil_thrashing(const bp::object& obj, unsigned maxiter)
{
  PyEval_InitThreads();
  ECTO_SCOPED_GILRELEASE();
  unsigned nthread = boost::thread::hardware_concurrency();

  for (unsigned j=0; j<maxiter; ++j)
    serv.post(boost::bind(&call_back_to_python, obj));

  for (unsigned j=0; j<nthread; ++j)
    {
      tgroup.create_thread(boost::bind(&boost::asio::io_service::run, boost::ref(serv)));
    }

  tgroup.join_all();
}

