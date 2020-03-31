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
#include <ecto/python.hpp>
#include <boost/foreach.hpp>
#include <ecto/plasm.hpp>
#include <ecto/scheduler.hpp>

namespace ecto
{
  namespace bp = boost::python;

  struct Executer
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<plasm::ptr>("plasm", "A plasm to execute.").required(true);
      p.declare<int>("niter", "Number of iterations.", 0);
      p.declare<bp::object>("inputs", "A python dict of inputs. {'in_name':cellinst,...}");
      p.declare<bp::object>("outputs", "A python dict of outputs. {'out_name':cellinst,...}");
    }

    static void
    extract(const tendrils& p, tendrils& ts, const std::string& name, tendrils cell::*member)
    {
      bp::object obj;
      p[name] >> obj;
      if (!obj || obj == bp::object())
        return;
      bp::list l = bp::dict(obj).items();
      for (int j = 0, end = bp::len(l); j < end; ++j)
      {
        bp::object key = l[j][0];
        bp::object value = l[j][1];
        std::string keystring = bp::extract<std::string>(key);
        ecto::cell_ptr cell = bp::extract<ecto::cell_ptr>(bp::getattr(value,"__impl"));
        tendril_ptr t = ((*cell).*member)[keystring];
        ts.declare(keystring, t);
      }

    }

    static void
    declare_io(const tendrils& p, tendrils& in, tendrils& out)
    {
      extract(p, in, "inputs", &cell::inputs);
      extract(p, out, "outputs", &cell::outputs);
    }

    void
    configure(const tendrils& p, const tendrils& /*in*/, const tendrils& /*out*/)
    {
      p["plasm"] >> plasm_;
      plasm_->configure_all();
      sched_.reset(new scheduler(plasm_));
      niters = p["niter"];
    }

    int
    process(const tendrils& /*in*/, const tendrils& /*out*/)
    {
      if (*niters > 0)
        sched_->execute(*niters);
      else
        sched_->execute(0);
      return ecto::OK;
    }
    plasm::ptr plasm_;
    boost::shared_ptr<scheduler> sched_;
    spore<int> niters;
  };
}
ECTO_DEFINE_MODULE(ecto_X)
{
}
ECTO_CELL(ecto_X, ecto::Executer, "Executer", "Executes a plasm.");
