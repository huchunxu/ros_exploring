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
#include <Python.h>
#include <boost/python.hpp>
#include <ecto/tendrils.hpp>
#include <boost/exception/diagnostic_information.hpp>

namespace bp = boost::python;

using namespace ecto;

TEST(TendrilTest, MakeTendril)
{
  Py_Initialize();
  tendril_ptr tp = make_tendril<bp::object>();
  EXPECT_TRUE(tp->get<bp::object>() == bp::object());
}

TEST(TendrilTest, Dirtiness)
{
  {
    tendril meh;
    EXPECT_FALSE(meh.dirty());
  }

  tendril t(0.5f, "docstring");

  EXPECT_EQ(t.type_name(), "float");
  EXPECT_EQ(t.doc(), "docstring");

  EXPECT_FALSE(t.dirty());
  EXPECT_EQ(t.get<float>(), 0.5f);
  EXPECT_FALSE(t.dirty());
  t << 0.75f;
  EXPECT_FALSE(t.dirty());
  EXPECT_EQ(t.get<float>(), 0.75f);

  t.dirty(true);
  EXPECT_TRUE(t.dirty());
  t.notify();
  EXPECT_FALSE(t.dirty());
}

TEST(TendrilTest, Constructors)
{
  {
    tendril meh;
    EXPECT_FALSE(meh.dirty());
    EXPECT_FALSE(meh.user_supplied());
    EXPECT_FALSE(meh.has_default());
    EXPECT_TRUE(meh.is_type<tendril::none>());
  }

  {
    tendril meh(0.5f, "docstring");
    EXPECT_FALSE(meh.dirty());
    EXPECT_FALSE(meh.user_supplied());
    EXPECT_TRUE(meh.has_default());
    EXPECT_TRUE(meh.is_type<float>());

    meh << 2.0f;

    EXPECT_TRUE(meh.has_default());
    EXPECT_TRUE(meh.is_type<float>());

    EXPECT_TRUE(meh.get<float>()==2.0f);

    meh.user_supplied(true);
    meh.dirty(true);
    //user_supplied and dirty are explicit states.
    EXPECT_TRUE(meh.user_supplied());
    EXPECT_TRUE(meh.dirty());

    meh.notify();
    EXPECT_TRUE(meh.user_supplied());
    EXPECT_FALSE(meh.dirty());
  }
  {
    tendril_ptr meh = make_tendril<float>();
    EXPECT_FALSE(meh->dirty());
    EXPECT_FALSE(meh->user_supplied());
    EXPECT_FALSE(meh->has_default());
    meh << 2.0f;
    EXPECT_TRUE(meh->get<float>()==2.0f);
    EXPECT_FALSE(meh->has_default());
    EXPECT_FALSE(meh->user_supplied());
    EXPECT_FALSE(meh->dirty());
  }
}

TEST(TendrilTest, NonPointerNess)
{
  tendril a(0.5f, "A float"), b, c;
  b = a;
  c = a;
  c << 3.14f;
  EXPECT_NE(a.get<float>(),c.get<float>());
  EXPECT_EQ(a.get<float>(),b.get<float>());
  EXPECT_NE(&a.get<float>(),&c.get<float>());
  EXPECT_NE(&a.get<float>(),&b.get<float>());
}

TEST(TendrilTest, Copyness)
{
  tendril a(0.5f, "A float"), b, c;
  b << a;
  c << b;
  c << 3.14f;
  EXPECT_NE(a.get<float>(),c.get<float>());
  EXPECT_EQ(a.get<float>(),b.get<float>());
  EXPECT_NE(&a.get<float>(),&c.get<float>());
  EXPECT_NE(&a.get<float>(),&b.get<float>());
  //self copy should be ok
  c << a;

}

TEST(TendrilTest, Typeness)
{
  tendril a(0.5f, "A float"), b(0.5, "A double."), c;
  EXPECT_THROW(b << a, except::TypeMismatch);
  EXPECT_NO_THROW(c = a);
  EXPECT_THROW(c << b, except::TypeMismatch);
  EXPECT_NO_THROW(c = b);
  EXPECT_THROW(c << a,except::TypeMismatch);
  tendril n1(tendril::none(),"A none"), n2(tendril::none(),"Another none"), n3;
  n2 << n1;
  n1 << n2;
  n1 = n2;
  n2 = n1;
  n1 << b;
  n2 << n1;
  EXPECT_EQ(n2.get<double>(),0.5);
  EXPECT_THROW(n2 << a, except::TypeMismatch);
  //EXPECT_THROW(b.copy_value(n3), except::ValueNone);
}

TEST(TendrilTest, AssignmentOfPODS)
{
  tendril a(0.005f, "A float");
  EXPECT_TRUE(a.is_type<float>());
  EXPECT_EQ(a.get<float>(), 0.005f);

  tendril b(500.0, "some double");
  EXPECT_TRUE(b.is_type<double>());
  EXPECT_EQ(b.get<double>(), 500.0);

  tendril c;
  EXPECT_TRUE(c.is_type<tendril::none>());

  // nothing is mutated on throwing conversion
  EXPECT_THROW(b << a, except::TypeMismatch);
  EXPECT_TRUE(a.is_type<float>());
  EXPECT_EQ(a.get<float>(), 0.005f);
  EXPECT_TRUE(b.is_type<double>());
  EXPECT_EQ(b.get<double>(), 500.0);

  // assignee takes on the type and value of the assigned 
  EXPECT_NO_THROW(c = a);
  EXPECT_TRUE(a.is_type<float>());
  EXPECT_EQ(a.get<float>(), 0.005f);
  EXPECT_TRUE(c.is_type<float>());
  EXPECT_EQ(c.get<float>(), 0.005f);

  // again nothing is mutated on throwing conversion
  EXPECT_THROW(c << b, except::TypeMismatch);
  EXPECT_TRUE(b.is_type<double>());
  EXPECT_EQ(b.get<double>(), 500.0);
  EXPECT_TRUE(c.is_type<float>());
  EXPECT_EQ(c.get<float>(), 0.005f);

  // again assignment suceeds
  EXPECT_NO_THROW(c = b);
  EXPECT_TRUE(b.is_type<double>());
  EXPECT_EQ(b.get<double>(), 500.0);
  EXPECT_TRUE(c.is_type<double>());
  EXPECT_EQ(c.get<double>(), 500.0);

  // same as above
  EXPECT_THROW(c << a,except::TypeMismatch);
}

TEST(TendrilTest, AssignmentOfNone)
{
  tendril n1(tendril::none(),"A none");
  tendril n2(tendril::none(),"Another none");
  
  tendril n3;
  n2 << n1;
  EXPECT_EQ(n1.get<tendril::none>(), n2.get<tendril::none>());
  n1 << n2;
  EXPECT_EQ(n1.get<tendril::none>(), n2.get<tendril::none>());

  n1 = n2;
  n2 = n1;

  tendril _500(500.0, "five hundred");

  n1 << _500;
  EXPECT_TRUE(n1.is_type<double>());
  EXPECT_TRUE(_500.is_type<double>());
  EXPECT_EQ(n1.get<double>(), 500.0);
  EXPECT_EQ(_500.get<double>(), 500.0);
  
}

TEST(TendrilTest, Python2PODConversion)
{
  tendril bpt(bp::object(2.05), "A bp object");
  tendril dt(7.05, "A double");

  // you can't do this... first you get out the bp::object,
  // then you get the double out of it.
  EXPECT_THROW(bpt.get<double>(),
               except::TypeMismatch);

  // tendril(bp::object) autoconverts from tendril(nonobject)
  dt << bpt;
  EXPECT_EQ(2.05, dt.get<double>());

  // can be overwritten thereafter
  dt << 7.05;
  EXPECT_EQ(dt.get<double>(), 7.05);

  // copyee is not mutated
  double value = bp::extract<double>(bpt.get<bp::object>());
  EXPECT_EQ(value, 2.05);

  // dt has not lost its internal type
  EXPECT_THROW( dt << std::string("NOTADOUBLE"), except::TypeMismatch);

  // but we can't autoconvert from None
  bpt << bp::object();
  try {
    dt << bpt;
    FAIL();
  } catch (except::FailedFromPythonConversion& tm) {
    std::cout << diagnostic_information(tm) << "\n";
  }
}

TEST(TendrilTest, POD2PythonConversion)
{
  tendril bpt(bp::object(2.05), "A bp object");
  tendril dt(7.05, "A double");

  {
    bp::extract<double> extractor(bpt.get<bp::object>());
    EXPECT_EQ(extractor(), 2.05);
  }
  EXPECT_TRUE(bpt.is_type<bp::object>());
  bpt << dt;

  // bpt is still a bp::object
  EXPECT_TRUE(bpt.is_type<bp::object>());

  // dt is still a double
  EXPECT_TRUE(dt.is_type<double>());

  {
    // double was copied correctly into dt
    bp::extract<double> extractor(bpt.get<bp::object>());
    EXPECT_EQ(extractor(), 7.05);
  }
  // dt was not mutated
  EXPECT_EQ(dt.get<double>(), 7.05);

}

TEST(TendrilTest, BoostPyDefaultness)
{
  tendrils ts;
  ts.declare<bp::object>("x","A bp object");
  bp::object x;
  ts["x"] >> x;

  if(x == bp::object())
  {
    std::cout << "x is none" << std::endl;
  }
}

TEST(TendrilTest, SyntacticSugar)
{
  int x = 2;
  float y = 3.14;
  std::string z = "z";

  tendril tx(x,"doc"),ty(y,"doc"),tz(z,"doc");
  tz >> z;
  tz << z;
  ty >> y;
  ty << y;
  tx << x;
  tx >> x;
  EXPECT_THROW(tx >> y;, except::TypeMismatch);
  EXPECT_THROW(tx << z;, except::TypeMismatch);


  tendrils ts;
  ts.declare<int>("x");
  ts.declare<float>("y");
  ts.declare<std::string>("z");
  ts["x"] >> x;
  ts["x"] << x;
  ts["y"] >> y;
  ts["y"] << y;
  ts["z"] >> z;
  ts["z"] << z;
  EXPECT_THROW(ts["z"] >> y;, except::TypeMismatch);
  EXPECT_THROW(ts["z"] << x;, except::TypeMismatch);

  EXPECT_THROW(ts["w"] >> x;, except::NonExistant);
  EXPECT_THROW(ts["t"] << x;, except::NonExistant);

}

TEST(TendrilTest, Nones)
{

  tendril_ptr a = make_tendril<tendril::none>();
  tendril_ptr b = make_tendril<tendril::none>();
  EXPECT_TRUE(a->is_type<tendril::none>());
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));
  a << b;
  std::cout << "a type: " << a->type_name() << "\n";
  std::cout << "b type: " << b->type_name() << "\n";
  EXPECT_TRUE(a->is_type<tendril::none>());
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));
  a >> b;
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));

  // you can assign anything to a none tendril, it changes type
  a << 7.05;
  EXPECT_TRUE(a->is_type<double>());
  EXPECT_EQ(a->get<double>(), 7.05);

  // note: now a is a double, you can't assign a string to it
  std::string s("ess");
  EXPECT_THROW(a << s, except::TypeMismatch);

  // assignment makes it a vanilla none again
  a = b;
  EXPECT_TRUE(a->is_type<tendril::none>());
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));

  // bp object with a string in it
  bp::object obj(s);
  
  a << obj;
  EXPECT_TRUE(a->is_type<bp::object>());
}

TEST(TendrilTest, ConversionTableFromNoneColumn)
{
  tendril none_;

  { // none << none
    tendril othernone_;
    othernone_ << none_;
  }

  { // object << none
    tendril object_(bp::object(3.14159), "pyobj");
    EXPECT_THROW(object_ << none_, except::ValueNone);
  }

  { // double << none
    tendril double_(3.14159, "double");
    EXPECT_THROW(double_ << none_, except::ValueNone);
  }
}


TEST(TendrilTest, ConversionTableFromPyObjectColumn)
{
  tendril pypi_(bp::object(3.1415), "py pi");

  { // none << object
    tendril none_;
    none_ << pypi_;
    bp::object rt = none_.get<bp::object>();
    bp::extract<double> extractor(rt);
    EXPECT_EQ(extractor(), 3.1415);
  }

  { // object << object
    tendril o2(bp::object(7.777), "sevens");
    o2 << pypi_;
    bp::object rt = o2.get<bp::object>();
    bp::extract<double> extractor(rt);
    EXPECT_EQ(extractor(), 3.1415);
  }

  { // double << object (compatible)
    tendril double_(5.555, "double");
    double_ << pypi_;
    EXPECT_EQ(double_.get<double>(), 3.1415);
  }

  { // double << object (incompatible)
    tendril string_(std::string("oops"), "double");
    try {
      string_ << pypi_;
      FAIL();
    } catch (except::FailedFromPythonConversion &e) {
      std::cout << diagnostic_information(e) << "\n";
    }
  }
}

TEST(TendrilTest, ConversionTableFromUDTColumn)
{
  tendril udt_(std::string("STRINGY"), "py pi");

  { // none << udt
    tendril none_;
    none_ << udt_;
    std::string s = none_.get<std::string>();
    EXPECT_EQ(s, "STRINGY");
  }

  { // object << udt
    tendril o2(bp::object(7.777), "sevens");
    o2 << udt_;
    bp::object rt = o2.get<bp::object>();
    std::string xtracted = bp::extract<std::string>(rt);
    EXPECT_EQ(xtracted, std::string("STRINGY"));
  }

  { // string << udt (compatible)
    tendril string_(std::string("NOTSTRINGY"), "is other string");
    string_ << udt_;
    EXPECT_EQ(string_.get<std::string>(), std::string("STRINGY"));
    // not the same string
    EXPECT_NE(&(string_.get<std::string>()), &(udt_.get<std::string>()));
  }

  { // double << udt (incompatible)
    tendril double_(3.1415, "double");
    EXPECT_THROW(double_ << udt_, except::TypeMismatch);
  }
}


TEST(TendrilTest, ConvertersCopied)
{
  tendril_ptr a = make_tendril<tendril::none>();
  tendril_ptr b = make_tendril<double>();
  EXPECT_FALSE(a->same_type(*b));
  EXPECT_FALSE(b->same_type(*a));
  *a = *b;
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));
  bp::object obj(3.1415);
  *a << obj;
  EXPECT_EQ(a->get<double>(), 3.1415);
  bp::object obj2;
  *a >> obj2;
  bp::extract<double> e(obj2);
  EXPECT_TRUE(e.check());
  EXPECT_EQ(e(), 3.1415);
}


TEST(TendrilTest, ConvertersCopied2)
{
  tendril_ptr a = make_tendril<tendril::none>();
  tendril_ptr b = make_tendril<double>();
  *a << *b; //copy the converters
  bp::object o(2.0);
  *a << o; //copy boost python object, should try to convert to double.
  EXPECT_EQ(name_of<double>(), a->type_name());
  EXPECT_EQ(2.0, a->get<double>());
}

TEST(TendrilTest, Nullptr)
{
  tendril_ptr a, b;
  try
  {
    a << b;
  } catch (ecto::except::NullTendril& e)
  {
    std::cout << boost::diagnostic_information(e) << "\n";
  }
}
namespace testy{
  struct FooBar{
    int x;
  };
}
TEST(TendrilTest, TypeReg)
{
  ecto::make_tendril<testy::FooBar>();
  tendril
    a = ecto::registry::tendril::get("int"),
    b = ecto::registry::tendril::get("testy::FooBar"),
    c = ecto::registry::tendril::get("std::string");
  tendril ta(int(4),""), tb(testy::FooBar(), ""), tc(std::string(""),"");
  EXPECT_TRUE(ta.same_type(a));
  EXPECT_TRUE(tb.same_type(b));
  EXPECT_TRUE(tc.same_type(c));
}
