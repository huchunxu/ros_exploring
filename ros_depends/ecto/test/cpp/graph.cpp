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

// This file is use in the docs
#include <Python.h>
#include <gtest/gtest.h>
#include <ecto/ecto.hpp>
#include <ecto/scheduler.hpp>
#include <ecto/plasm.hpp>

#define STRINGDIDLY(A) std::string(#A)

using namespace ecto;
namespace {
  // Declare a bogus cell that just outputs stuff
  struct Module1
  {
    static void
    declare_io(const tendrils& p, tendrils& in, tendrils& out)
    {
      out.declare<double> ("d");
    }
  };

  // Declare a bogus cell that just reads stuff
  struct Module2
  {
    static void
    declare_io(const tendrils& p, tendrils& in, tendrils& out)
    {
      in.declare<double> ("d");
    }
  };

  // Declare a bogus cell that gets stuff in and spits it out
  struct Passthrough
  {
    static void declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      // The type could be explicit but the 'none' is used to specify any type
      in.declare<tendril::none>("in", "Any type");
      out.declare<tendril::none>("out", "Any type on the output...");
      out["out"] = in["in"]; //assign the ptr.
    }
  };

}
TEST(Plasm, Viz)
{
  // Create an empty plasm
  ecto::plasm p;
  // Create some cells
  ecto::cell::ptr m1(new ecto::cell_<Module1>), m2(new ecto::cell_<Module2>);
  // Initialize the cells
  m1->declare_params();
  m1->declare_io();
  m2->declare_params();
  m2->declare_io();
  // Connect the cells in the plasm: the output named 'd' of the cell 'm1' is
  // connected to the input 'd' of the cell 'm2'
  p.connect(m1,"d",m2,"d");
  // Visualize the plasm
  std::cout << p.viz() << std::endl;
}

TEST(Plasm, Passthrough)
{
  // Create an empty plasm
  ecto::plasm::ptr p(new ecto::plasm);
  // Create some cells
  ecto::cell::ptr m1(new cell_<Module1>),
    m2(new cell_<Module2>),
    pass(new cell_<Passthrough>);
  // Initialize some cells
  m1->declare_params();
  m2->declare_params();
  pass->declare_params();
  m1->declare_io();
  m2->declare_io();
  pass->declare_io();
  // Impose the output of a cell to be of certain value
  // That could be done inside the cell but this is just
  // to show that tendrils can be modified like in Python
  m1->outputs["d"] << 5.0;
  // Create some connection inside the plasm
  p->connect(m1,"d",pass,"in");
  p->connect(pass,"out",m2,"d");
  // Create a scheduler to execute your plasm
  ecto::scheduler sched(p);
  // Execute your plasm once
  sched.execute(1);
  double out;
  // Read the output 'd' of the 'm2' cell
  m2->inputs["d"] >> out;
  EXPECT_TRUE(out == 5.0);
  // Read the output 'out' of the plasm
  pass->outputs["out"] >> out;
  EXPECT_TRUE(out == 5.0);
}

TEST(Plasm, Registry)
{
  // Create a cell that is of type "Add" from the module "ecto_test"
  ecto::cell::ptr add = ecto::registry::create("ecto_test::Add");
  add->declare_params();
  add->declare_io();

  EXPECT_TRUE(add);
  // Display the name of the cell
  std::cout << add->name() << std::endl;
  EXPECT_EQ("ecto_test::Add", add->name());

  ecto::plasm::ptr p(new ecto::plasm());
  add->inputs["left"] << 1.0;
  add->inputs["right"] << 2.0;

  p->insert(add);
  ecto::scheduler sched(p);
  sched.execute(1);
  EXPECT_EQ(add->outputs.get<double>("out"), 3);
}



