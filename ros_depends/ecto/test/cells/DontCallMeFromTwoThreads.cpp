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
#include <boost/asio.hpp>

using ecto::tendrils;
namespace ecto_test
{
  struct DontCallMeFromTwoThreads
  {
    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<double> ("in");
      outputs.declare<double> ("out");
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
      boost::mutex::scoped_try_lock lock(mtx);

      if (! lock.owns_lock())
        throw std::runtime_error("we should have that damned lock.");

      // wait a bit so's we can be sure there will be collisions
      ecto::test::random_delay();
      usleep(1000);
      
      double value = inputs.get<double> ("in");
      // do yer thing
      outputs.get<double> ("out") = value;

      // unrock
      ECTO_LOG_DEBUG("this=%p done with the lock", this);
      return ecto::OK;

    }
    static boost::mutex mtx;
  };
  boost::mutex DontCallMeFromTwoThreads::mtx;

}
ECTO_CELL(ecto_test, ecto_test::DontCallMeFromTwoThreads, "DontCallMeFromTwoThreads",
          "Throws if process called concurrently from two threads.");

