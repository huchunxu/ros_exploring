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
#include <boost/date_time/posix_time/posix_time.hpp>

using ecto::tendrils;
namespace ecto_test
{
  namespace pt = boost::posix_time;

  struct Metrics
  {
    ecto::spore<pt::ptime> in;
    ecto::spore<double> hz, latency_seconds;
    ecto::spore<unsigned> queue_size;
    std::deque<pt::ptime> times;

    static void declare_params(tendrils& parameters) 
    { 
      parameters.declare<unsigned>("queue_size", "size of window to collect statistics over", 10);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<pt::ptime> ("in", "input");
      outputs.declare<double>("hz");
      outputs.declare<double>("latency_seconds");
    }

    void configure(const tendrils& parameters, const tendrils& inputs, const tendrils& outputs)
    {
      queue_size = parameters["queue_size"];
      hz = outputs["hz"];
      latency_seconds = outputs["latency_seconds"];
      in = inputs["in"];
    }

    int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      pt::ptime now(pt::microsec_clock::universal_time());
      times.push_back(now);
      if (times.size() > *queue_size)
        times.pop_front();

      *latency_seconds = (now - *in).total_microseconds() / 1e+6;
      std::cout << "Got message which is " << (now - *in) << " old.\n";
      pt::time_duration queuedur = now - times.front();
      double thishz = (1e+6 / queuedur.total_microseconds()) * (times.size()-1);
      *hz = thishz;
      std::cout << "hz=" << thishz << "\n";

      return 0;
    }
  };
}

ECTO_CELL(ecto_test, ecto_test::Metrics, "Metrics", 
            "Calcluate bandwidth and latency based on timestamps received from an upstream ping.");

