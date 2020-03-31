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
#include <boost/asio.hpp>
#include <ecto/all.hpp>
#include <ecto/plasm.hpp>
#include <ecto/atomic.hpp>
#include <boost/exception/diagnostic_information.hpp>

using namespace ecto;

TEST(Strands,ValueType)
{
  ecto::strand s, t, u, v;
  ASSERT_FALSE(s == t);
  ASSERT_FALSE(s == u);
  ASSERT_FALSE(s == v);
  ASSERT_FALSE(t == u);
  ASSERT_FALSE(t == v);
  ASSERT_FALSE(u == v);

  t = u;

  ASSERT_FALSE(s == t);
  ASSERT_FALSE(s == u);
  ASSERT_FALSE(s == v);
  ASSERT_TRUE(t == u);
  ASSERT_FALSE(t == v);
  ASSERT_FALSE(u == v);

  t = v;

  ASSERT_FALSE(s == t);
  ASSERT_FALSE(s == u);
  ASSERT_FALSE(s == v);
  ASSERT_FALSE(t == u);
  ASSERT_TRUE(t == v);
  ASSERT_FALSE(u == v);

  ecto::strand copied(t);
  
  ASSERT_FALSE(s == t);
  ASSERT_FALSE(s == u);
  ASSERT_FALSE(s == v);
  ASSERT_FALSE(t == u);
  ASSERT_TRUE(t == v);
  ASSERT_TRUE(copied == t);
  ASSERT_TRUE(copied == v);
  ASSERT_FALSE(u == v);

}

namespace {

  boost::mutex mtx;
  boost::asio::io_service s;

  struct Crashy
  {
    boost::asio::deadline_timer dt;

    Crashy() : dt(s) { }
    int process(const ecto::tendrils&, const ecto::tendrils&)
    {
      boost::mutex::scoped_try_lock lock(mtx);
      ECTO_ASSERT(lock.owns_lock(), "we should own this lock");
      dt.expires_from_now(boost::posix_time::milliseconds(200));
      dt.wait();
      return ecto::OK;
    }
  };
}
ECTO_THREAD_UNSAFE(Crashy);

#if 0
TEST(Strands, Crashy_is_ECTO_THREAD_UNSAFE)
{
  ecto::plasm::ptr p(new ecto::plasm);
  for (unsigned j=0; j<10; ++j) {
    ecto::cell::ptr m(new cell_<Crashy>);
    p->insert(m);
  }

  ecto::schedulers::multithreaded sched(p);
  sched.execute(5);
}

TEST(Strands, Crashy2_is_on_user_supplied_strand)
{
  ecto::plasm::ptr p(new ecto::plasm);
  ecto::strand s;
  ecto::cell_ptr prev = ecto::registry::create_initialized("ecto_test::Generate<double>");
  prev->name("gen");
  for (unsigned j=0; j<10; ++j) {
    ecto::cell::ptr m(ecto::registry::create_initialized("ecto_test::DontCallMeFromTwoThreads"));
    m->strand_ = s;
    p->connect(prev, "out", m, "in");
    prev = m;
  }

  ecto::schedulers::multithreaded sched(p);
  sched.execute(5);
  std::cout << " made it here *************" << std::endl;
}
#endif

namespace
{
  ecto::atomic<int> n_concurrent(0), max_concurrent(0);

  struct NotCrashy
  {
    boost::asio::deadline_timer dt;

    NotCrashy() : dt(s) { }
    int process(const ecto::tendrils&, const ecto::tendrils&)
    {
      {
        ecto::atomic<int>::scoped_lock nconc(n_concurrent), maxconc(max_concurrent);
        ++nconc.value;
        if (maxconc.value < nconc.value)
          maxconc.value = nconc.value;
      }

      dt.expires_from_now(boost::posix_time::milliseconds(200));
      dt.wait();

      {
        ecto::atomic<int>::scoped_lock nconc(n_concurrent);
        --nconc.value;
      }
      return ecto::OK;
    }
  };

}


#if 0
TEST(Strands, ConcurrencyCount)
{
  ecto::plasm::ptr p(new ecto::plasm);
  for (unsigned j=0; j<10; ++j) {
    ecto::cell::ptr m(new cell_<NotCrashy>);
    p->insert(m);
  }

  ecto::schedulers::multithreaded sched(p);
  sched.execute(boost::thread::hardware_concurrency()+8);
  ecto::atomic<int>::scoped_lock cur_con(n_concurrent), max_con(max_concurrent);
  ASSERT_EQ(cur_con.value, 0);
  ASSERT_EQ(max_con.value, boost::thread::hardware_concurrency());
  ECTO_LOG_DEBUG("max concurrent runs: %u", max_con.value);
}
#endif



TEST(Strands, Registry) {
  ecto::cell_ptr cp = ::ecto::registry::create("ecto_test::CantCallMeFromTwoThreads");
  ASSERT_TRUE(cp->strand_);

  ecto::cell_ptr cpother = ::ecto::registry::create("ecto_test::CantCallMeFromTwoThreads");
  ASSERT_TRUE(cpother->strand_);

  ASSERT_TRUE(*(cp->strand_) == *(cpother->strand_));

  ecto::cell_ptr cp2 = ::ecto::registry::create("ecto_test::DontCallMeFromTwoThreads");
  ASSERT_FALSE(cp2->strand_);


}
