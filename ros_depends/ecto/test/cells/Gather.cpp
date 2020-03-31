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
#include <boost/foreach.hpp>
#include <boost/format.hpp>

using ecto::tendrils;
namespace ecto_test
{
  template<typename ValueT>
  struct Gather
  {
    typedef ValueT value_type;

    static void declare_params(ecto::tendrils& p)
    {
      p.declare<int> ("n", "N to gather", 2);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      int n = parameters.get<int> ("n");
      for (int ii = 0; ii < n; ii++)
        {
          inputs.declare<value_type> (str(boost::format("in_%04d") % ii),
                                      "An " + ecto::name_of<value_type>() + "input.");
        }
      outputs.declare<value_type> ("out", "The sum of all inputs.");
    }

    void configure(const tendrils& parameters, const tendrils& inputs, const tendrils& outputs)
    {
      n_ = parameters.get<int> ("n");
    }

    int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      //SHOW();
      value_type& out = outputs.get<value_type> ("out");
      out = 0;
      typedef std::pair<std::string, ecto::tendril_ptr> pp;
      BOOST_FOREACH(const pp& in,inputs)
        {
          out += in.second->get<value_type> ();
        }
      return ecto::OK;
    }

    int n_;
  };
}

ECTO_CELL(ecto_test, ecto_test::Gather<int>, "Gather", "Gather ints");
ECTO_CELL(ecto_test, ecto_test::Gather<double>, "Gather_double", "Gather doubles");

