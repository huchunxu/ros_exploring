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
#include <iostream>

namespace hello_ecto
{

  using ecto::tendrils;

  struct Printer
  {
    static void declare_params(tendrils& params)
    {
      params.declare<std::string> ("str", "The default string to print", "hello");
    }

    static void declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      in.declare<std::string> ("str", "The string to print.", parms.get<std::string> ("str"));
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      str_ = inputs["str"];
    }

    int process(const tendrils& in, const tendrils& /*out*/)
    {
      std::cout << *str_ << std::endl;
      return ecto::OK;
    }
    ecto::spore<std::string> str_;
  };

  struct Reader
  {
    static void declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      out.declare<std::string> ("output", "Output from standard in");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      output_ = outputs["output"];
    }

    int process(const tendrils& in, const tendrils& out)
    {
      std::string s;
      std::cin >> s;
      *output_  = s;
      if(s == "q")
        return ecto::QUIT;
      return ecto::OK;
    }
    ecto::spore<std::string> output_;
  };

}

ECTO_DEFINE_MODULE(hello_ecto)
{ }

ECTO_CELL(hello_ecto, hello_ecto::Printer, "Printer", "Prints a string input to standard output.");
ECTO_CELL(hello_ecto, hello_ecto::Reader, "Reader", "Reads input from standard input.");

