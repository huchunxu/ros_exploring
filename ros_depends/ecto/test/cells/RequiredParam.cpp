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
#include <ecto/spore.hpp>
#include <ecto/registry.hpp>

using ecto::tendrils;
using ecto::spore;

namespace ecto_test
{
  struct RequiredParam
  {
    spore<double> x_, in_, out_;
    static void declare_params(tendrils& params)
    {
      params.declare<double> ("x", "A required parameter.").required(true);
    }

    static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      in.declare<double> ("in", "an input", 2.1253);
      out.declare<double> ("out", "The input + x", 0.0);
    }

    void configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      x_ = params["x"];
      in_ = in["in"];
      out_ = out["out"];
    }

    int process(const tendrils& in, const tendrils& out)
    {
      *out_ = *x_ + *in_;
      return 0;
    }

  };

}

ECTO_CELL(ecto_test, ecto_test::RequiredParam, "RequiredParam", "Required parameter")
;

