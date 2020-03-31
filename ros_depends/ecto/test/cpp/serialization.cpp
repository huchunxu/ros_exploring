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
#include <ecto/serialization/registry.hpp>
#include <ecto/serialization/cell.hpp>

#include <boost/serialization/vector.hpp>
#include <fstream>
#include <boost/foreach.hpp>

#pragma GCC diagnostic ignored "-Wsign-compare"

TEST(SerialTest, float_test)
{
  {
    ecto::tendril t(1.0f, "A tendril.");
    std::ofstream out("float_tendril.ecto");
    boost::archive::binary_oarchive oa(out);
    oa & t;
  }
  {
    ecto::tendril t;
    std::ifstream in("float_tendril.ecto");
    boost::archive::binary_iarchive ia(in);
    ia & t;
    std::cout << t.type_name() << " " << t.get<float>() << " " << t.doc() << std::endl;
    EXPECT_EQ(t.get<float>(), 1.0f);
    EXPECT_EQ(t.doc(), "A tendril.");

  }
}

TEST(SerialTest, many_tendrils_test)
{
  {
    std::vector<ecto::tendril> ts(10);
    ts[0] << std::string("hello");
    ts[0].set_doc("A docstr 0");
    ts[1] << std::string("you boy");
    ts[1].set_doc("A docstr 1");

    ts[2] << 4;
    ts[2].set_doc("4 is an int");

    ts[3] << 4.0;
    ts[3].set_doc("4.0 is a double");
    std::ofstream out("many.ecto");
    boost::archive::binary_oarchive oa(out);
    oa & ts;
  }
  {
    std::vector<ecto::tendril> ts;
    std::ifstream in("many.ecto");
    boost::archive::binary_iarchive ia(in);
    ia & ts;
    for (size_t i = 0; i < ts.size(); i++)
    {
      std::cout << ts[i].type_name() << " " << ts[i].doc() << "\n";
    }
  }
}

ECTO_REGISTER_SERIALIZERS(double);

TEST(SerialTest, tendrils_test)
{
  {
    ecto::tendrils ts;
    ts.declare<std::string>("whoopie", "A docstr for woopie", "pie");
    ts.declare<int>("foo", "A foo.", 18);
    std::ofstream out("tendrils.ecto");
    boost::archive::binary_oarchive oa(out);
    oa & ts;
  }
  {
    ecto::tendrils ts;
    std::ifstream in("tendrils.ecto");
    boost::archive::binary_iarchive ia(in);
    ia & ts;
    BOOST_FOREACH(ecto::tendrils::value_type x, ts)
        {
          std::cout << x.first << " : " << x.second->type_name() << std::endl;
        }EXPECT_EQ(ts.size(), 2);
    int foo;
    ts["foo"] >> foo;
    EXPECT_EQ(foo, 18);
    std::string whoopie;
    ts["whoopie"] >> whoopie;
    EXPECT_EQ(whoopie, "pie");
  }
}
namespace FooT
{
  struct BarStruct
  {
    int x;
  };
}
TEST(SerialTest, test_unknown)
{
  ecto::tendril t(FooT::BarStruct(), "An unknown type.");
  std::ofstream out("unknown.ecto");
  boost::archive::binary_oarchive oa(out);
  try{
    oa & t;
  }catch(std::exception& e) {
    std::cout << e.what() << std::endl;
    EXPECT_EQ(std::string("Could not find a serializer registered for the type: FooT::BarStruct"),e.what());
  }
}

TEST(SerialTest, Cell)
{
  {
    ecto::cell::ptr add = ecto::registry::create("ecto_test::Add");
    add->declare_params();
    add->declare_io();
    EXPECT_TRUE(add);
    std::cout << add->name() << std::endl;
    std::ofstream out("add.ecto");
    boost::archive::binary_oarchive oa(out);
    oa & add;
  }

  {
    ecto::cell::ptr add;
    std::ifstream in("add.ecto");
    boost::archive::binary_iarchive ia(in);
    ia & add;
    std::cout << add->type() << std::endl;
    EXPECT_TRUE(add);
    EXPECT_EQ(add->type(),"ecto_test::Add");
  }

}

TEST(SerialTest, CellWithTendrilWithCell)
{
  {
    ecto::cell::ptr add = ecto::registry::create("ecto_test::Add");
    add->declare_params();
    add->declare_io();
    add->outputs["out"] << 3.14;

    ecto::cell::ptr If = ecto::registry::create("ecto::If");
    If->declare_params();
    If->declare_io();
    If->parameters["cell"] << add;

    std::ofstream out("If.ecto");
    boost::archive::binary_oarchive oa(out);
    oa & If;
  }
  {
    ecto::cell::ptr add,If;
    std::ifstream in("If.ecto");
    boost::archive::binary_iarchive ia(in);
    ia & If;
    If->parameters["cell"] >> add;
    std::cout << add->type() << std::endl;
    EXPECT_TRUE(add);
    EXPECT_EQ(add->type(),"ecto_test::Add");
    double out;
    add->outputs["out"] >> out;
    EXPECT_EQ(out,3.14); //check that we survived.
    If->inputs["__test__"] << true;
    If->process();
    std::cout << "Post process." << std::endl;
    If->parameters["cell"] >> add;
    add->outputs["out"] >> out;
    EXPECT_EQ(out,0); //check that we ran. 0+0=0
  }
}

TEST(SerialTest, Plasm)
{
  {
    ecto::plasm::ptr p(new ecto::plasm);
    ecto::cell::ptr gen = ecto::registry::create("ecto_test::Generate<double>"),
      add = ecto::registry::create("ecto_test::Add");
    gen->name("gen");
    gen->declare_params();
    gen->declare_io();
    add->name("add");
    add->declare_params();
    add->declare_io();

    p->connect(gen, "out", add, "left");
    p->connect(gen, "out", add, "right");
    ecto::scheduler sched(p);
    sched.execute(2);
    std::cout << add->outputs.get<double>("out") << std::endl;
    EXPECT_EQ(2, gen->outputs.get<double>("out"));
    EXPECT_EQ(4, add->outputs.get<double>("out"));
    std::ofstream out("graph.ecto");
    boost::archive::binary_oarchive oa(out);
    oa & *p;
  }
  {
    std::cout << "deserial." << std::endl;
    ecto::plasm::ptr p(new ecto::plasm());
    std::ifstream in("graph.ecto");
    boost::archive::binary_iarchive ia(in);
    ia & *p;
    std::vector<ecto::cell::ptr> cells = p->cells();

    EXPECT_EQ(2, cells.size());
    std::map<std::string, double> results;
    for (size_t i = 0; i < cells.size(); i++)
    {
      std::cout << cells[i]->name() << " out="
                << cells[i]->outputs.get<double>("out") << std::endl;

      results[cells[i]->name()] = cells[i]->outputs.get<double>("out");
    }

    EXPECT_EQ(results.count("add"), 1);
    EXPECT_EQ(results.count("gen"), 1);
    //expect results.
    EXPECT_EQ(4, results["add"]);
    EXPECT_EQ(2, results["gen"]);
  }

}
