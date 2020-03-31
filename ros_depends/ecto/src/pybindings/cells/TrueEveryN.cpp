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
  struct TrueEveryN
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<int>("n", "Will be true at every iteration where count%n == 0", 2);
      p.declare<int>("count", "Initial value of counter, will be incremented at every call to process.", 0);

    }
    static void
    declare_io(const tendrils& p, tendrils& in, tendrils& out)
    {
      out.declare<bool>("flag");
    }
    void
    configure(const tendrils&p, const tendrils&in, const tendrils&out)
    {
      n_ = p["n"];
      count_ = p["count"];
      flag_ = out["flag"];
    }
    int
    process(const tendrils& in, const tendrils& out)
    {
      *flag_ = ((*count_)++ % (*n_) == 0);
      return ecto::OK;
    }
    spore<bool> flag_;
    spore<int> count_, n_;
  };
}

ECTO_CELL(cells, ecto::TrueEveryN, "TrueEveryN", "Will give a true result every n executions.");
