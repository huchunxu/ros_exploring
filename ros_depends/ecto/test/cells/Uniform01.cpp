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
#include <boost/random.hpp>
#include <boost/scoped_ptr.hpp>

using ecto::tendrils;

namespace ecto_test
{
  struct Uniform01
  {
    struct impl {

      boost::mt19937 generator;
      boost::uniform_real<> uniform_dist;
      boost::variate_generator<boost::mt19937&, boost::uniform_real<> > uni;

      impl(unsigned seed) 
        : generator(seed), uniform_dist(0,1), uni(generator, uniform_dist)
      { }

      double operator()() {
        return uni();
      }
    };


    boost::scoped_ptr<impl> pimpl_;
    ecto::spore<double> out_;

    unsigned ncalls;

    static void declare_params(tendrils& parameters)
    {
      parameters.declare<unsigned> ("seed", "Seed.  By default the RNG is seeded from the system time.");
      parameters.declare<unsigned> ("ncalls", "Call this many times, return only the last. "
                                    "Used to generate CPU load in testing.", 1);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      outputs.declare<double> ("out", "output");
    }

    void configure(const tendrils& parameters, const tendrils& inputs, const tendrils& outputs)
    {
      out_ = outputs["out"];
      ecto::spore<unsigned> seed_ = parameters["seed"];
      // if the user supplied a seed, use it, else use the system time
      if (seed_.user_supplied()) 
        pimpl_.reset(new impl(*seed_));
      else
        pimpl_.reset(new impl(static_cast<unsigned>(std::time(0))));
      ncalls=parameters.get<unsigned>("ncalls");
    }

    int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      for (unsigned j=0; j<ncalls; ++j)
        *out_ = (*pimpl_)();
      return ecto::OK;
    }
  };
}

ECTO_CELL(ecto_test, ecto_test::Uniform01, "Uniform01", "Generate random doubles uniformly distributed on [0, 1)");

