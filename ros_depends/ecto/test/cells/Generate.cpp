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
#include <ecto/registry.hpp>

using ecto::tendrils;
namespace ecto_test
{
  template<typename T>
  struct Generate
  {
    ecto::spore<T> step_, start_, stop_, out_; //so that it receives updates dynamically.

    static void declare_params(tendrils& parameters)
    {
      parameters.declare<T> ("step", "The step with which i generate integers.", 2);
      parameters.declare<T> ("start", "My starting value.", 0);
      parameters.declare<T> ("stop", "Stop if the generated value exceeds this upper bound (0 implies no upper bound)", 0);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      outputs.declare<T> ("out", "The starting value + (step * iterations).");
    }

    void configure(const tendrils& parameters, const tendrils& inputs, const tendrils& outputs)
    {
      start_ = parameters["start"];
      step_ = parameters["step"];
      stop_ = parameters["stop"];
      out_ = outputs["out"];
      *out_ = *start_ - *step_;
    }

    int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      if (( *stop_ != 0 ) && ( *out_ + *step_ > *stop_ )) {
        return ecto::QUIT;
      }
      *out_ += *step_;
      return ecto::OK;
    }
  };
}

ECTO_CELL(ecto_test, ecto_test::Generate<double>, "Generate", "Generate doubles");

