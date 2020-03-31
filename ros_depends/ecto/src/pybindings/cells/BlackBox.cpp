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

#include <ecto/ecto.hpp>
#include <boost/foreach.hpp>
#include <ecto/plasm.hpp>
#include <ecto/scheduler.hpp>

#include "../tendril_spec.hpp"
namespace ecto
{
  namespace py
  {
    namespace bp = boost::python;

    struct BlackBox
    {
      typedef scheduler scheduler_t;
      static void
      shallow_merge(const tendrils& fts, tendrils& ts)
      {
        std::string key;
        tendril_ptr t;
        BOOST_FOREACH(boost::tie(key,t), fts)
            {
              t->required(false);
              ts.declare(key, t);
            }
      }

      int
      process(const tendrils& /*in*/, const tendrils& /*out*/)
      {
        //FIXME
        //TODO Scheduler is a pain here. Need to expose as a scope so that exceptions are informative.
        if (!sched_)
        {
          try
          {
            plasm_->configure_all();

          } catch (ecto::except::EctoException& e)
          {
            throw std::runtime_error(ecto::except::diagnostic_string(e));
          }
          sched_.reset(new scheduler_t(plasm_));
        }
        try
        {
          if (niter_ > 0)
            sched_->execute(niter_);
          else
            sched_->execute(0);
          if (! sched_->running())
            return ecto::QUIT;
        } catch (ecto::except::EctoException& e)
        {
          throw std::runtime_error(ecto::except::diagnostic_string(e));
        }
        return ecto::OK;
      }
      void stop(){
        //std::cout << "Stopped! " << __PRETTY_FUNCTION__ << std::endl;

      }
      plasm::ptr plasm_;
      boost::shared_ptr<scheduler_t> sched_;
      int niter_;
    };

    cell::ptr
    create_black_box(plasm::ptr plasm, int niter, const tendrils& p, const tendrils& i, const tendrils& o)
    {
      cell_<BlackBox>::ptr black_box(new cell_<BlackBox>);
      cell::ptr base(black_box);
      base->declare_params(); //declare params and IO
      base->declare_io();
      BlackBox::shallow_merge(p, base->parameters);
      BlackBox::shallow_merge(i, base->inputs);
      BlackBox::shallow_merge(o, base->outputs);
      base->configure(); //This causes the impl to be created
      black_box->impl().plasm_ = plasm; //initialize a few thangs
      black_box->impl().niter_ = niter;
      return black_box;
    }
  }
}

namespace ecto
{
  namespace py
  {
    using bp::arg;
    void
    wrap_black_box()
    {
      bp::def("create_black_box", create_black_box, //fnc
              (arg("plasm"), arg("niter"), arg("parameters"), //
              arg("inputs"), arg("outputs")), //args
              "Constructs a BlackBox." //doc str
              );
    }
  }
}
