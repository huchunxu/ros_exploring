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
#include <ecto/rethrow.hpp>
#include <ecto/python.hpp>
#include <ecto/scheduler.hpp>

namespace ecto {
  namespace except {
    namespace py {

      boost::exception_ptr rethrowable_in_interpreter_thread;

      int rethrow_in_python(void * val)
      {
        ECTO_LOG_DEBUG("rethrowing: %p", val);
        // handle_exception does the translation to python.
        // rethrow_exception turns the exception_ptr back in to an
        // exception.  since this is "scheduled" (do it ASAP) to be
        // called by the interpreter, our exception gets translated
        // and rethrown there.
        boost::python::handle_exception(boost::bind<void>(&boost::rethrow_exception, rethrowable_in_interpreter_thread));
        return -1;
      }


      //
      //  Get the current exception (as set by a catch and boost::current_exception)
      //  and schedule a rethrow in the interpreter thread.
      //
      void rethrow_schedule()
      {
        ECTO_START();
        // some of our tests are @ the c++ api level, no interpreter.
        // In this case don't
        if (!Py_IsInitialized()) {
          ECTO_LOG_DEBUG("%s", "Python not initialized, rethrowing");
          boost::rethrow_exception(boost::current_exception());
        }
        {
          ECTO_SCOPED_CALLPYTHON();

          ECTO_LOG_DEBUG("%s", "rethrow scheduled");
          rethrowable_in_interpreter_thread = boost::current_exception();
          Py_AddPendingCall(&rethrow_in_python, (void*)13);
        }
        ECTO_FINISH();
      }

      void rethrow (boost::function<void()> h)
      {
        ECTO_START();
        try {
          h();
        } catch (const boost::exception&) {
          // serv.stop();
          rethrow_schedule();
          //throw;
        } catch (const std::exception&) {
          rethrow_schedule();
        }
        ECTO_FINISH();
      }

      //
      //  Get the current exception (as set by a catch and boost::current_exception)
      //  and schedule a rethrow in an asio service
      //
      void rethrow_schedule(boost::asio::io_service& serv)
      {
        ECTO_START();
        serv.dispatch(boost::bind<void>(&boost::rethrow_exception, boost::current_exception()));
        ECTO_FINISH();
      }

      void rethrow (boost::function<void()> h, boost::asio::io_service& serv, ecto::scheduler* sched)
      {
        ECTO_START();
        try {
          boost::asio::io_service::work work(serv);
          h();
        } catch (const boost::exception&) {
          // serv.stop();
          ECTO_LOG_DEBUG("rethrower stopping scheduler at %p", sched);
          rethrow_schedule(serv);
          if (sched) sched->stop();
        } catch (const std::exception&) {
          ECTO_LOG_DEBUG("rethrower stopping scheduler at %p", sched);
          rethrow_schedule(serv);
          if (sched) sched->stop();
        }
        ECTO_FINISH();
      }
    }
  }
}

