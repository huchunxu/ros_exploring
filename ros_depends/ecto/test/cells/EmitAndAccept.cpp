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
#include <ecto/tendril.hpp>

using ecto::tendrils;
namespace ecto_test
{
  template <typename T>
  struct Emit
  {
    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      outputs.declare<T> ("output", "output");
    }

    int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      outputs.get<T>("output") = T();
      return ecto::OK;
    }
  };

  template <typename T>
  struct Accept
  {
    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<T> ("input", "input");
    }

    int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      inputs.get<T>("input");
      return ecto::OK;
    }
  };
  
  struct Struct { float x, y, z; };
}


#define DECLS(T)                                                        \
  ECTO_CELL(ecto_test, ecto_test::Emit<T>,                              \
            "Emit_" # T, "Emit_" # T);                                  \
  ECTO_CELL(ecto_test, ecto_test::Accept<T>,                            \
            "Accept_" # T, "Accept_" # T);

DECLS(bool);
DECLS(int);
DECLS(float);

using ecto_test::Struct;
DECLS(Struct);

using std::string;
DECLS(string);

typedef ecto::tendril::none none;
DECLS(none);
