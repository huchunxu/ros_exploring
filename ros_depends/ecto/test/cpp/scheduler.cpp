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
// TODO: These should be in some shared location.
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

plasm::ptr make_quitter_plasm(unsigned N = 1)
{
  plasm::ptr p(new plasm);

  cell::ptr generator(registry::create("ecto_test::Generate<double>"));
  generator->declare_params();
  generator->declare_io();

  cell::ptr quitter(registry::create("ecto_test::QuitAfter"));
  quitter->declare_params();
  quitter->parameters["N"] << N;
  quitter->declare_io();

  p->connect(generator, "out", quitter, "in");
  return p;
}

plasm::ptr make_thrower_plasm(unsigned N = 1)
{
  plasm::ptr p(new plasm);

  cell::ptr generator(registry::create("ecto_test::Generate<double>"));
  generator->declare_params();
  generator->declare_io();

  cell::ptr thrower(registry::create("ecto_test::ThrowAfter"));
  thrower->declare_params();
  thrower->parameters["N"] << N;
  thrower->declare_io();

  p->connect(generator, "out", thrower, "in");
  return p;
}

typedef std::map<std::string, cell::ptr> cell_map_t;
struct plasm_state {
  plasm::ptr pPlasm;
  cell_map_t cellMap;
};

plasm_state make_counter_plasm(unsigned N = 1, cell::ptr counter = cell::ptr())
{
  plasm_state ps;
  ps.pPlasm.reset(new plasm);

  cell::ptr generator(registry::create("ecto_test::Generate<double>"));
  generator->declare_params();
  generator->declare_io();
  ps.cellMap.insert(make_pair(generator->name(), generator));

  cell::ptr quitter = registry::create("ecto_test::QuitAfter");
  quitter->declare_params();
  quitter->declare_io();
  ps.cellMap.insert(make_pair(quitter->name(), quitter));
  quitter->parameters["N"] << N;

  if (! counter) {
    counter = registry::create("ecto_test::StartStopCounter");
    counter->declare_params();
    counter->declare_io();
    ps.cellMap.insert(make_pair(counter->name(), counter));
  }

  ps.pPlasm->connect(generator, "out", quitter, "in");
  ps.pPlasm->connect(quitter, "out", counter, "in");

  return ps;
}

plasm_state make_do_over_plasm(unsigned N = 5)
{
  plasm_state ps;
  ps.pPlasm.reset(new plasm);

  cell::ptr generator(registry::create("ecto_test::Generate<double>"));
  generator->declare_params();
  generator->declare_io();
  ps.cellMap.insert(make_pair(generator->name(), generator));

  cell::ptr do_over(registry::create("ecto_test::DoOverFor"));
  do_over->declare_params();
  do_over->parameters["N"] << N;
  do_over->declare_io();
  ps.cellMap.insert(make_pair(do_over->name(), do_over));

  cell::ptr counter = registry::create("ecto_test::StartStopCounter");
  counter->declare_params();
  counter->declare_io();
  ps.cellMap.insert(make_pair(counter->name(), counter));

  ps.pPlasm->connect(generator, "out", do_over, "in");
  ps.pPlasm->connect(do_over, "out", counter, "in");
  return ps;
}

} // End of anonymous namespace.

TEST(Scheduler, CreateAndDestroy)
{
  plasm::ptr p = makeplasm();
  scheduler* s = new scheduler(p);
  EXPECT_FALSE(s->running());
  delete s;
}


TEST(Scheduler, DestroyWhileRunning1)
{
  plasm::ptr p = makeplasm();
  scheduler* s = new scheduler(p);
  EXPECT_FALSE(s->running());
  s->prepare_jobs();
  EXPECT_TRUE(s->running());
  delete s;
}

TEST(Scheduler, DestroyWhileRunning2)
{
  plasm::ptr p = makeplasm();
  scheduler* s = new scheduler(p);
  EXPECT_FALSE(s->running());
  s->prepare_jobs();
  s->run_job();
  EXPECT_TRUE(s->running());
  delete s;
}

TEST(Scheduler, DestroyWhileRunning3)
{
  plasm::ptr p = makeplasm();
  scheduler* s = new scheduler(p);
  EXPECT_FALSE(s->running());
  s->execute(1);
  EXPECT_TRUE(s->running());
  delete s;
}

TEST(Scheduler, StopAndDestroy)
{
  plasm::ptr p = makeplasm();
  scheduler* s = new scheduler(p);
  EXPECT_FALSE(s->running());
  s->execute(2);
  EXPECT_TRUE(s->running());
  s->stop();
  EXPECT_FALSE(s->running());
  delete s;
}

TEST(Scheduler, TimedRun)
{
  using namespace boost::posix_time;
  const time_duration eps = microseconds(300);
  const double dur_secs = 1./(1000*1000);
  for (unsigned i = 0; i < 7; ++i)
  {
    const unsigned dur_usecs = static_cast<unsigned>(std::pow(10., i));
    plasm::ptr p = makeplasm(dur_secs);
    scheduler s(p);
    EXPECT_FALSE(s.running());
    s.prepare_jobs();
    EXPECT_TRUE(s.running());
    const ptime start = microsec_clock::universal_time();
    s.run(dur_usecs);
    const ptime end = microsec_clock::universal_time();
    
    const time_duration dur = end - start;
    EXPECT_GE(dur, microseconds(dur_usecs));
    EXPECT_LE(dur, microseconds(dur_usecs) + eps);
  }
}

TEST(Scheduler, State1)
{
  // Test states with execute() and explicit stop().
  plasm::ptr p = makeplasm(0.001);
  scheduler s(p);
  EXPECT_EQ(s.state(), scheduler::INIT);
  s.execute(5);
  EXPECT_EQ(s.state(), scheduler::RUNNING);
  s.stop();
  EXPECT_EQ(s.state(), scheduler::FINI);
  s.stop(); // Make sure stop() after explicit stop works.
  EXPECT_EQ(s.state(), scheduler::FINI);
  s.prepare_jobs(1); // Make sure it can start back up.
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
}

TEST(Scheduler, State2)
{
  // Test states with prepare_jobs() and explicit stop().
  plasm::ptr p = makeplasm(0.001);
  scheduler s(p);
  EXPECT_EQ(s.state(), scheduler::INIT);
  s.prepare_jobs(5);
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
  s.run_job();
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
  s.run();
  EXPECT_EQ(s.state(), scheduler::RUNNING);
  s.stop();
  EXPECT_EQ(s.state(), scheduler::FINI);
  s.stop(); // Make sure stop() after explicit stop works.
  EXPECT_EQ(s.state(), scheduler::FINI);
  s.prepare_jobs(1); // Make sure it can start back up.
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
}

TEST(Scheduler, State3)
{
  // Test states with execute() and automatic stop().
  plasm::ptr p = make_quitter_plasm(1); // Quit after 1 full iteration.
  scheduler s(p);
  EXPECT_EQ(s.state(), scheduler::INIT);
  s.execute(1);
  EXPECT_EQ(s.state(), scheduler::RUNNING);
  s.execute(1);
  EXPECT_EQ(s.state(), scheduler::FINI);
  s.stop(); // Make sure stop() after automatic stop works.
  EXPECT_EQ(s.state(), scheduler::FINI);
  s.prepare_jobs(1); // Make sure it can start back up.
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
}

TEST(Scheduler, State4)
{
  // Test states with execute() and automatic stop().
  plasm::ptr p = make_quitter_plasm(1); // Quit after 1 full iteration.
  scheduler s(p);
  EXPECT_EQ(s.state(), scheduler::INIT);
  s.execute(); // Go till stop.
  EXPECT_EQ(s.state(), scheduler::FINI);
  s.stop(); // Make sure stop() after automatic stop works.
  EXPECT_EQ(s.state(), scheduler::FINI);
  s.prepare_jobs(1); // Make sure it can start back up.
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
}

TEST(Scheduler, State5)
{
  // Test states with prepare_jobs() and automatic stop().
  plasm::ptr p = make_quitter_plasm(1); // Quit after 1 full iteration.
  scheduler s(p);
  EXPECT_EQ(s.state(), scheduler::INIT);
  s.prepare_jobs(1);
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
  s.run();
  EXPECT_EQ(s.state(), scheduler::RUNNING);
  s.prepare_jobs(1);
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
  s.run();
  EXPECT_EQ(s.state(), scheduler::FINI);
  s.prepare_jobs(1); // Make sure it can start back up.
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
}

TEST(Scheduler, State6)
{
  // Test states with prepare_jobs() and automatic stop().
  plasm::ptr p = make_quitter_plasm(1); // Quit after 1 full iteration.
  scheduler s(p);
  EXPECT_EQ(s.state(), scheduler::INIT);
  s.prepare_jobs(); // Go till stop.
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
  s.run();
  EXPECT_EQ(s.state(), scheduler::FINI);
  s.prepare_jobs(1); // Make sure it can start back up.
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
}

TEST(Scheduler, State7)
{
  // Test states with execute() and exception.
  plasm::ptr p = make_thrower_plasm(1); // Quit after 1 full iteration.
  scheduler s(p);
  EXPECT_EQ(s.state(), scheduler::INIT);
  s.execute(1); // Won't throw on the first iter.
  EXPECT_EQ(s.state(), scheduler::RUNNING);
  EXPECT_THROW(s.execute(1), except::CellException);
  EXPECT_EQ(s.state(), scheduler::ERROR);
  s.stop(); // Make sure stop() after exception works.
  EXPECT_EQ(s.state(), scheduler::ERROR);
  s.prepare_jobs(1); // Make sure it can start back up.
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
}

TEST(Scheduler, State8)
{
  // Test states with execute() and exception.
  plasm::ptr p = make_thrower_plasm(1); // Quit after 1 full iteration.
  scheduler s(p);
  EXPECT_EQ(s.state(), scheduler::INIT);
  EXPECT_THROW(s.execute(), except::CellException);
  EXPECT_EQ(s.state(), scheduler::ERROR);
  s.stop(); // Make sure stop() after automatic stop works.
  EXPECT_EQ(s.state(), scheduler::ERROR);
  s.prepare_jobs(1); // Make sure it can start back up.
  EXPECT_EQ(s.state(), scheduler::EXECUTING);
}

TEST(Scheduler, TickCheck1)
{
  plasm_state ps1 = make_counter_plasm(5); // Quit after 5 iterations.
  scheduler s1(ps1.pPlasm);

  cell::ptr counter = ps1.cellMap["ecto_test::StartStopCounter"];
  ASSERT_EQ(! counter, false);

  const unsigned & nprocess = counter->outputs.get<unsigned>("nprocess");
  EXPECT_EQ(nprocess, 0);
  s1.execute(); // Go till stop.
  EXPECT_EQ(nprocess, 5);
}

TEST(Scheduler, TickCheck2)
{
  plasm_state ps1 = make_counter_plasm(5); // Quit after 5 iterations.
  scheduler s1(ps1.pPlasm);
  
  cell::ptr counter = ps1.cellMap["ecto_test::StartStopCounter"];
  ASSERT_EQ(! counter, false);

  plasm_state ps2 = make_counter_plasm(5, counter); // Also quit after 5 iterations.
  scheduler s2(ps2.pPlasm);

  const unsigned & nprocess = counter->outputs.get<unsigned>("nprocess");
  EXPECT_EQ(nprocess, 0);

  for (size_t n = 0; n < 5; ++n) {
    s1.execute(); // Go till stop.
    const unsigned expect1 =  5 + n*10;
    EXPECT_EQ(nprocess, expect1);
    s2.execute(); // Go till stop.
    const unsigned expect2 = 10 + n*10;
    EXPECT_EQ(nprocess, expect2);
  }
}

TEST(Scheduler, DoOver)
{
  plasm_state ps = make_do_over_plasm(5); // 
  scheduler s(ps.pPlasm);

  cell::ptr do_over = ps.cellMap["ecto_test::DoOverFor"];
  ASSERT_EQ(! do_over, false);
  const unsigned & current = do_over->outputs.get<unsigned>("current");
  EXPECT_EQ(current, 0);

  cell::ptr counter = ps.cellMap["ecto_test::StartStopCounter"];
  ASSERT_EQ(! counter, false);
  const unsigned & nprocess = counter->outputs.get<unsigned>("nprocess");
  EXPECT_EQ(nprocess, 0);

  for (size_t n = 0; n < 5; ++n) {
    s.execute(1); // Go till stop.
    EXPECT_EQ(nprocess, n+1);
    EXPECT_EQ(current, 5+n*5);
  }
}
