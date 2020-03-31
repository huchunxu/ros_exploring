//
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
#include <Python.h>
#include <gtest/gtest.h>

#include <ecto/ecto.hpp>
#include <ecto/plasm.hpp>
#include <ecto/scheduler.hpp>

#include <boost/date_time/microsec_time_clock.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

using namespace ecto;

namespace {

plasm::ptr makeplasm(double secs = 0.1)
{
  plasm::ptr p(new plasm);

  cell::ptr ping(registry::create("ecto_test::Ping"));
  ping->declare_params();
  ping->declare_io();

  cell::ptr sleep0(registry::create("ecto_test::Sleep"));
  cell::ptr sleep1(registry::create("ecto_test::Sleep"));
  sleep0->declare_params();
  sleep0->parameters["seconds"] << secs;
  sleep0->declare_io();

  sleep1->declare_params();
  sleep1->parameters["seconds"] << secs;
  sleep1->declare_io();

  p->connect(ping, "out", sleep0, "in");
  p->connect(sleep0, "out", sleep1, "in");
  return p;
}

typedef std::map<std::string, cell::ptr> cell_map_t;
struct plasm_state {
  plasm::ptr pPlasm;
  cell_map_t cellMap;
};

plasm_state make_shared_sleep_plasm(double secs = 0.1, cell::ptr sleep = cell::ptr())
{
  plasm_state ps;
  ps.pPlasm.reset(new plasm);

  cell::ptr ping(registry::create("ecto_test::Ping"));
  ping->declare_params();
  ping->declare_io();
  ps.cellMap.insert(make_pair(ping->name(), ping));

  if (! sleep) {
    sleep = registry::create("ecto_test::Sleep");
    sleep->declare_params();
    sleep->parameters["seconds"] << secs;
    sleep->declare_io();
    ps.cellMap.insert(make_pair(sleep->name(), sleep));
  }

  ps.pPlasm->connect(ping, "out", sleep, "in");

  return ps;
}

} // End of anonymous namespace.

TEST(Profile, Stats1)
{
  plasm::ptr p = makeplasm();
  scheduler s(p);
  std::cout << s.stats();
  s.execute(5);
  std::cout << s.stats();
}

TEST(Profile, Stats2)
{
  plasm::ptr p = makeplasm();
  scheduler s(p);
  for (size_t n = 0; n < 5; ++n)
    s.execute(1);
  std::cout << s.stats();
}

TEST(Profile, Stats3)
{
  plasm::ptr p = makeplasm(.5);
  scheduler s(p);
  s.execute(1);
  std::cout << s.stats();
}

TEST(Profile, Stats4)
{
  plasm_state ps1 = make_shared_sleep_plasm(0.1);
  scheduler s1(ps1.pPlasm);
  
  cell::ptr sleep = ps1.cellMap["ecto_test::Sleep"];
  ASSERT_EQ(! sleep, false);

  plasm_state ps2 = make_shared_sleep_plasm(0.1, sleep);
  scheduler s2(ps2.pPlasm);

  for (size_t n = 0; n < 5; ++n) {
    s1.execute(1); // Go till stop.
    s2.execute(1); // Go till stop.
  }

// Make sure that stats() calls don't fail any assertions.
std::cerr << s1.stats() << "\n";
std::cerr << s2.stats() << "\n";
}
