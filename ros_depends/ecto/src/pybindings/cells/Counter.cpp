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
namespace ecto
{
  struct Counter
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<unsigned>("count", "Initial value of counter, will be incremented at every call to process.", 0);
      p.declare<unsigned>("every","print every this many frames", std::numeric_limits<unsigned>::max());
    }
    static void
    declare_io(const tendrils& p, tendrils& in, tendrils& out)
    {
      in.declare<tendril::none>("input","Any input, counts the number of executions.");
      out.declare<unsigned>("count","The count of input.", p.get<unsigned>("count"));
    }
    void
    configure(const tendrils&p, const tendrils&in, const tendrils&out)
    {
      count_ = out["count"];
      every_ = p["every"];
    }
    int
    process(const tendrils& /*in*/, const tendrils& /*out*/)
    {
      ++(*count_);
      if (*count_ % *every_ == 0)
        std::cout << "Counter: " << *count_ << "\n";
      return ecto::OK;
    }
    spore<unsigned> count_, every_;
  };
}

ECTO_CELL(cells, ecto::Counter, "Counter", 
          "Gives an execution count. Useful for counting the number of times that an output of another cell"
          " is valid.  Prints progress every so many frames.");
