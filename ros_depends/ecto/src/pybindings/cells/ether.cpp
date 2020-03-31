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
#include <ecto/all.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/python/overloads.hpp>
namespace bp = boost::python;
namespace ecto
{
  struct EtherSource
  {
  };

  struct EtherSink
  {
    static void
    declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      in.declare<tendril::none>("in", "Any type");
    }
  };

  bp::tuple
  entangled_pair(tendril_ptr value,const std::string& source_name="EntangledSource", 
                 const std::string& sink_name = "EntangledSink")
  {
    bp::tuple p;
    cell::ptr source(new cell_<EtherSource>), 
      sink(new cell_<EtherSink>);

    source->declare_params();
    source->declare_io();
    source->name(source_name);

    sink->declare_params();
    sink->declare_io();
    sink->name(sink_name);
    sink->inputs["in"] << *value;
    source->outputs.declare("out",sink->inputs["in"]);
    p = bp::make_tuple(source, sink);
    return p;
  }
  BOOST_PYTHON_FUNCTION_OVERLOADS(entangled_pair_overloads, entangled_pair, 1,3)
  namespace py
  {
    using bp::arg;
    void
    wrap_ether()
    {
      bp::def("EntangledPair", entangled_pair,
              entangled_pair_overloads((arg("value"), arg("source_name"), arg("sink_name")), //args
                  "Constructs a pair of entangled cells. Useful for " //
                  "teleportation of tendrils without constructing edges in a graph."//doc str
                  ));
    }
  }
}


