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
#include <boost/asio.hpp>

using ecto::tendrils;
namespace ecto_test
{
  struct Increment
  {
    double amount_;
    unsigned delay_ms_;
    static void declare_params(ecto::tendrils& p)
    {
      p.declare<double> ("amount", "Amount to increment by.", 1.0);
      p.declare<unsigned> ("delay", "How long it takes to increment in milliseconds", 0);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<double> ("in", "input");
      outputs.declare<double> ("out", "output");
    }

    void configure(const tendrils& parms, const tendrils& inputs, const tendrils& outputs)
    {
      amount_ = parms.get<double> ("amount");
      delay_ms_ = parms.get<unsigned> ("delay");
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
      if (delay_ms_ > 0)
        {
          boost::asio::io_service s;
          boost::asio::deadline_timer dt(s);
          dt.expires_from_now(boost::posix_time::milliseconds(delay_ms_));
          dt.wait();
        }
      double in = inputs.get<double> ("in");
      double result = in + amount_;
      // std::cout << this << " incrementer: " << in << " ==> " << result << std::endl;
      outputs.get<double> ("out") = result;
      return ecto::OK;
    }
  };
}

ECTO_CELL(ecto_test, ecto_test::Increment, "Increment", "Increment input by some amount");

