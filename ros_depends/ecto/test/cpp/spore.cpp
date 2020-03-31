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
#include <gtest/gtest.h>
#include <ecto/ecto.hpp>


using namespace ecto;

TEST(SporeTest, LifeTime)
{
  {
    spore<double> d = make_tendril<double>();
    EXPECT_TRUE(d);
  }
  {
    spore<double> d;
    EXPECT_FALSE(d);
    EXPECT_ANY_THROW(*d);

    d = make_tendril<double>();
    EXPECT_TRUE(d);
    *d = 3.555;
    EXPECT_EQ(*d, 3.555);
    // reassign
    d = make_tendril<double>();
    ASSERT_NE(*d, 3.555);

    EXPECT_ANY_THROW(d = make_tendril<std::string>());
  }
}

TEST(SporeTest, NoDefault)
{
  tendril_ptr p = make_tendril<double>();
  spore<double> d = p; //p has to stay in scope...
  EXPECT_FALSE(d.user_supplied());
  EXPECT_FALSE(d.dirty());
  EXPECT_FALSE(d.has_default());

  *d = 3.14;
  EXPECT_FALSE(d.dirty());
  EXPECT_FALSE(d.user_supplied());
  EXPECT_FALSE(d.has_default());

  d.set_default_val(10);
  EXPECT_TRUE(d.has_default());
}

TEST(SporeTest, Default)
{
  tendril_ptr p = make_tendril<double>();
  EXPECT_FALSE(p->dirty());

  spore<double> d = p; //p has to stay in scope...
  EXPECT_FALSE(d.dirty());

  d.set_default_val(1.41421356);

  EXPECT_FALSE(d.user_supplied());
  EXPECT_FALSE(d.dirty());
  EXPECT_TRUE(d.has_default());

  EXPECT_EQ(*d, 1.41421356);
  EXPECT_FALSE(d.dirty());
  d.notify();
  EXPECT_FALSE(d.dirty());

  *d = 3.14;
  EXPECT_FALSE(d.dirty());
  EXPECT_FALSE(d.user_supplied());
  EXPECT_TRUE(d.has_default());
}

template<typename T>
struct cbs
{
  cbs() : count(0), val(0) { }

  void operator()(const T& new_val)
  {
    val = new_val;
    count++;
  }

  int count;
  T val;
};

TEST(SporeTest, Callbacks)
{
  tendril_ptr p = make_tendril<double>();
  spore<double> d = p; //p has to stay in scope...
  d.set_default_val(1.41421356);

  cbs<double> c;
  d.set_callback(boost::ref(c));
  d.notify();
  EXPECT_EQ(c.count, 0);
  EXPECT_EQ(c.val, 0);

  *d = 3.14;
  d.dirty(true); //dirtiness is explicit
  d.notify();
  EXPECT_EQ(c.count, 1);
  EXPECT_EQ(c.val, 3.14);

  *d = 5.55;
  EXPECT_FALSE(d.dirty());
  // callback didn't fire
  EXPECT_EQ(c.count, 1);
  EXPECT_EQ(c.val, 3.14);
  d.notify();

  // it didn't fire cause it aint dirty
  EXPECT_EQ(c.count, 1);
  EXPECT_EQ(c.val, 3.14);

  d.dirty(true);
  // still didn't fire... not notified
  EXPECT_EQ(c.count, 1);
  EXPECT_EQ(c.val, 3.14);

  // notifed, and was dirty... now it fires
  d.notify();
  EXPECT_EQ(c.count, 2);
  EXPECT_EQ(c.val, 5.55);
  EXPECT_FALSE(d.dirty());
}

TEST(SporeTest, Expressions)
{
  tendril_ptr ta = make_tendril<double>(),
    tb = make_tendril<double>(),
    tc = make_tendril<double>()
    ;

  spore<double> a(ta), b(tb), c(tc);
  *a = 13.;
  EXPECT_EQ(*a, 13.);

  *b = 14.;
  EXPECT_EQ(*b, 14.);

  *c = 15.;
  EXPECT_EQ(*c, 15.);

  *a = (*b + *c);
  
  EXPECT_EQ(*a, 29.);
  EXPECT_EQ(*b, 14.);
  EXPECT_EQ(*c, 15.);
}

struct SporeCellConst
{
  static void declare_params(tendrils&p)
  {
    p.declare<std::string>("foo");
  }
  static void declare_io(const tendrils& p, tendrils& i, tendrils& o)
  {

    i.declare<double>("d");
    o.declare<double>("out");
  }
  void configure(const tendrils& p,const tendrils& i,const tendrils& o)
  {
    foo = p["foo"];
    d = i["d"];
    out = o["out"];
    //shouldn't compile.
    //i.declare<double>("d","a new d.");
  }

  int process(const tendrils& i,const tendrils& o)
  {
    *out = *d;
//    out << d; //fail to compile
//    o["out"] << d;//fail to compile
    o["out"] << *d;
    o["out"] << 3.4;
    o["out"] << i["d"];

//    o["out"] = 2.0;//fail to compile.

//    tendril_ptr tp = out.get();//fail to compile

    //out >> d; //should fail to compile
    //i["d"] >> out; //should fail at compile time. FIXME this doesn't fail but should not compile.
    i["d"] >> *out; //should not fail.
    return ecto::OK;
  }
  spore<double> d,out;
  spore<std::string> foo;
};

TEST(SporeTest, Semantics)
{
  cell::ptr c1(new cell_<SporeCellConst>);
  c1->declare_params();
  c1->declare_io();
  c1->configure();
  c1->process();
}


TEST(SporeTest, DefaultConstruction)
{

  spore<double> d;
  //post conditions
  EXPECT_FALSE(d);
  EXPECT_TRUE(!d);

}


TEST(SporeTest, CopyConstructionDefault)
{
  spore<double> d1;

  spore<double> d2(d1);
  //post condition
  EXPECT_FALSE(d2);

  spore<double> d3 = d1;
  //post condition
  EXPECT_FALSE(d3);

  d1 = make_tendril<double>();
  EXPECT_TRUE(d1);
  EXPECT_FALSE(d3);
  EXPECT_FALSE(d2);


}


TEST(SporeTest, CopyConstructionValue)
{
  spore<double> d1( make_tendril<double>());

  spore<double> d2(d1);
  //post condition
  EXPECT_TRUE(d2);

  spore<double> d3 = d1;
  //post condition
  EXPECT_TRUE(d3);

  //all point to the same thing here.
  *d1 = 3.14;
  EXPECT_EQ(*d1,*d2);
  EXPECT_EQ(*d2,*d3);

  //assign a null spore to d2, noone else should be affected.
  d2 = spore<double>();
  EXPECT_TRUE(d1);
  EXPECT_TRUE(d3);
  EXPECT_FALSE(d2);
}


TEST(SporeTest, ImplicitConstructor)
{
  tendril_ptr d = make_tendril<double>();
  tendril_ptr s = make_tendril<std::string>();
  spore<double> dd;
  spore<std::string> ss;
  dd = d;
  ss = s;
  EXPECT_THROW(dd = s,ecto::except::TypeMismatch);
  EXPECT_THROW(ss = d,ecto::except::TypeMismatch);

}

TEST(SporeTest, NullAssign)
{
  spore<std::string> ss;
  EXPECT_THROW(
  ss = tendril_ptr();
  ,
  ecto::except::NullTendril);
}


