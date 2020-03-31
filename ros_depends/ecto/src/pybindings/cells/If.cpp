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
namespace ecto
{
  namespace bp = boost::python;

  struct If
  {
    static void declare_params(tendrils& p)
    {
      p.declare<cell::ptr>("cell", 
                           "Cell to conditionally execute."
                           " The inputs and outputs of this cell will be"
                           " replicated to the If cell.").required(true);
      p.declare<std::string>("input_tendril_name", 
                           "Name to use for the conditional input tendril.",
                           "__test__"
                           );
    }

    static void declare_io(const tendrils& p, tendrils& in, tendrils& out)
    {
      in.declare<bool>(p.get<std::string>("input_tendril_name"),
                       "The test value. If this is true then "
                       " cell::process() is called.", false);
      cell::ptr c;
      p["cell"] >> c;
      if(!c)
        return;//handle default well.
      in.insert(c->inputs.begin(), c->inputs.end());
      out.insert(c->outputs.begin(), c->outputs.end());
    }

    void configure(const tendrils& p, const tendrils& in, const tendrils& out)
    {
      p["cell"] >> c_;
      c_->configure();
      test_ = in[p.get<std::string>("input_tendril_name")];
    }

    int process(const tendrils& in, const tendrils& out)
    {
      if(*test_)
      {
        return c_->process();
      }
      return ecto::OK;
    }
    void start(){
      if(c_)
        c_->start();
    }
    void stop(){
      if(c_)
        c_->stop();
    }
    cell::ptr c_;
    spore<bool> test_;
  };
}

ECTO_CELL(cells, ecto::If, "If", "If true, process, else, don't.");
