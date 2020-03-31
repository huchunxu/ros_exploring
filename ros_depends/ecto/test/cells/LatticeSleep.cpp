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
#include <boost/random.hpp>

using ecto::tendrils;
using ecto::tendril;

namespace ecto_test
{
  namespace {
    boost::mt19937 generator;
    boost::uniform_real<> uniform_dist(0,0.1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > uni(generator, uniform_dist);
  }
  namespace pt = boost::posix_time;

  struct LatticeSleep
  {
    ecto::spore<double> sleep_sec;
    ecto::spore<pt::ptime> in, out;
    unsigned n;
    static void declare_params(tendrils& parameters)
    {
      parameters.declare<unsigned>("n", "number of ins and outs", 1);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      unsigned n = parameters.get<unsigned>("n");

      for (unsigned j=0; j<n; ++j)
        {
          inputs.declare<tendril::none>  ("in" + boost::lexical_cast<std::string>(j),  "input");
          outputs.declare<tendril::none> ("out" + boost::lexical_cast<std::string>(j), "output");
        }
    }

    void configure(const tendrils& parameters, const tendrils& inputs, const tendrils& outputs)
    {
      n = parameters.get<unsigned>("n");
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
      boost::this_thread::sleep(boost::posix_time::microseconds(int64_t(uni()*1.0e6)));
      for (unsigned j=0; j<n; ++j)
        {
          *outputs["out" + boost::lexical_cast<std::string>(j)] << *inputs["in" + boost::lexical_cast<std::string>(j)];
        }
      return 0;
    }
  };
}

ECTO_CELL(ecto_test, ecto_test::LatticeSleep, "LatticeSleep", 
          "Node with N inputs and N outputs, sleeps for a random period on each process()")

