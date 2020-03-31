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
#include <ecto/ecto.hpp>
#include <ecto/plasm.hpp>
#include <ecto/serialization/registry.hpp>
#include <ecto/serialization/cell.hpp>
#include <ecto/scheduler.hpp>
#include <cstring>
#include <fstream>
int
main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cout << argv[0] << " plasm.ecto NITER\n";
    return 1;
  }
  ecto::plasm::ptr p(new ecto::plasm());
  std::ifstream in(argv[1]);
  p->load(in);

  std::cout << "** graphviz" << std::endl;
  p->viz(std::cout);
  std::cout << std::endl;
  std::cout << "** cell listing" << std::endl;
  std::vector<ecto::cell::ptr> cells = p->cells();
  for (size_t i = 0; i < cells.size(); i++)
  {
    std::cout << cells[i]->name() << std::endl;
  }
  ecto::scheduler sched(p);
  return sched.execute(std::atoi(argv[2]));
}
