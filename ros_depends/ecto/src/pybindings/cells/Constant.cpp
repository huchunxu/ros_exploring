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

#include <Python.h>
#include <boost/python.hpp>
#include <ecto/ecto.hpp>
#include <iostream>
#include <queue>

namespace ecto
{
  namespace bp = boost::python;

  struct Constant
  {
    static void declare_params(tendrils& p)
    {
      p.declare(&Constant::value_, "value", "Value to output").required(true);
    }

    static void declare_io(const tendrils& params, tendrils& in, tendrils& out)
    {
      out.declare(&Constant::out_, "out", "Any type, constant.");
    }

    void configure(const ecto::tendrils& params,
                   const ecto::tendrils& /* in */,
                   const ecto::tendrils& out)
    {
      // initialise outputs so it can be used with directed configuration
      *out_ = *value_;
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      *out_ = *value_;

      return ecto::OK;
    }

    ecto::spore<bp::object> value_;
    ecto::spore<bp::object> out_;
  };
}

ECTO_CELL(cells, ecto::Constant, "Constant",
          "Constant node always outputs same value.");
