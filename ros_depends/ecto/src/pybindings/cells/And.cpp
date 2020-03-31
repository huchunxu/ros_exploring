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
#include <boost/format.hpp>

namespace ecto
{
  namespace bp = boost::python;

  struct And
  {
    static std::string get_input_string(unsigned int i)
    {
      return str(boost::format("in%i")%(i+1));
    }

    static void declare_params(tendrils& p)
    {
      p.declare<unsigned int>("ninput","Number of inputs to AND together",2);
    }

    static void declare_io(const tendrils& p, tendrils& in, tendrils& out)
    {
      unsigned int ninput = p.get<unsigned int>("ninput");
      //inputs
      for(unsigned int i=0; i<ninput; i++){
        in.declare<bool>(And::get_input_string(i),"A boolean input to be ANDed with the others",true);
      }

      //output
      out.declare<bool>("out","AND of the inputs");
    }

    void configure(const tendrils& p, const tendrils& in, const tendrils& out)
    {
      for(unsigned int i=0; i<in.size(); i++){
        inputs_.push_back(in[And::get_input_string(i)]);
      }
      output_ = out["out"];
    }

    int process(const tendrils& in, const tendrils& out)
    {
      *output_ = true;
      for(unsigned int i=0; i<inputs_.size(); i++){
        *output_ = *output_ && *inputs_[i];
      }
      return ecto::OK;
    }

    std::vector<spore<bool> > inputs_;
    spore<bool> output_;
  };
}

ECTO_CELL(cells, ecto::And, "And", "AND together some number of boolean inputs");
