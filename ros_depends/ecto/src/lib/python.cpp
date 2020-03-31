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
#include <boost/thread/mutex.hpp>
#include <ecto/python.hpp>
#include <ecto/python/repr.hpp>
#include <ecto/python/gil.hpp>
#include <ecto/log.hpp>

#include <deque>

namespace ecto {
  namespace py {

    gilstatus::gilstatus(const char* f, unsigned l, const char* w): file(f), line(l), what(w) { }

    bool operator==(const gilstatus& lhs, const gilstatus& rhs)
    {
      return lhs.file == rhs.file && lhs.line == rhs.line;
    }

    std::deque<gilstatus> gilstack;

    boost::mutex gilmutex;

    void showstack()
    {
      for(std::deque<gilstatus>::iterator iter = gilstack.begin(), end=gilstack.end();
          iter != end;
          ++iter)
        {
          ECTO_LOG_DEBUG("%s:%u %s", iter->file % iter->line % iter->what);
        }
    }


    std::string repr(const boost::python::object& obj)
    {
      return boost::python::extract<std::string>(obj.attr("__repr__")());
    }



    struct gil::impl : boost::noncopyable
    {
      PyGILState_STATE gstate;
    };

    gil::gil() : impl_(new gil::impl)
    {
      //impl_->gstate = PyGILState_Ensure();
    }

    gil::~gil()
    {
      //PyGILState_Release(impl_->gstate);
    }


    std::map<boost::thread::id, PyThreadState*> scoped_gil_release::thread_states = std::map<boost::thread::id, PyThreadState*>();

    scoped_gil_release::scoped_gil_release(const char* file, unsigned line)
      : mine(false), mystatus(file, line, "scoped_gil_release")
    {
      if (!Py_IsInitialized())
        return;
      boost::thread::id current_thread_id = boost::this_thread::get_id();
      if ( thread_states.find(current_thread_id) == thread_states.end() ) {
        thread_states[current_thread_id] = PyEval_SaveThread();
        mine = true;
      }
      {
        boost::unique_lock<boost::mutex> lock(gilmutex);
        gilstack.push_front(mystatus);
        showstack();
      }
    }

    scoped_gil_release::~scoped_gil_release() {
      if (!Py_IsInitialized())
        return;
      if (mine) {
        boost::thread::id current_thread_id = boost::this_thread::get_id();
        std::map<boost::thread::id, PyThreadState*>::iterator iter;
        iter = thread_states.find(current_thread_id);
        PyEval_RestoreThread(iter->second);
        thread_states.erase(iter);
        mine = false;
      }
      {
        boost::unique_lock<boost::mutex> lock(gilmutex);
        showstack();
        ECTO_ASSERT(gilstack.size() > 0, "There's no lock coords on the stack");
        ECTO_ASSERT(gilstack.front() == mystatus, "I can't pop a lock that isn't mine");
        gilstack.pop_front();
      }
    }

    scoped_call_back_to_python::scoped_call_back_to_python(const char* file, unsigned line)
      : have(false), mystatus(file, line, "scoped_call_python")
    {
      if (!Py_IsInitialized())
        return;

      have = true;
      gilstate = PyGILState_Ensure();
      {
        boost::unique_lock<boost::mutex> lock(gilmutex);
        gilstack.push_front(mystatus);
        showstack();
      }
    }

    scoped_call_back_to_python::~scoped_call_back_to_python()
    {
      if (!Py_IsInitialized())
        return;
      ECTO_ASSERT(have, "We have no GIL to release");
      PyGILState_Release(gilstate);
      {
        boost::unique_lock<boost::mutex> lock(gilmutex);
        showstack();
        ECTO_ASSERT(gilstack.size() > 0, "no lock to pop, ehm.");
        ECTO_ASSERT(gilstack.front() == mystatus, "can't pop a lock that isn't mine");
        gilstack.pop_front();
      }
    }

  }
}

