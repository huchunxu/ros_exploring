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

#include "plasm/impl.hpp"

namespace ecto {

  using namespace except;

  using graph::graph_t;
  using graph::edge;
  using graph::vertex;
  using graph::vertex_ptr;

  namespace
  {
    using boost::tie;
    using boost::add_vertex;

    graph::edge_ptr
    make_edge(const std::string& fromport, const std::string& toport)
    {
      graph::edge_ptr eptr(new graph::edge(fromport, toport));
      return eptr;
    }
  } // namespace


  plasm::impl::impl() {}

    //insert a cell into the graph, will retrieve the
    //vertex descriptor if its already in the graph...
  graph_t::vertex_descriptor
  plasm::impl::insert_module(cell_ptr m)
  {
    //use the vertex map to look up the graphviz descriptor (reverse lookup)
    ModuleVertexMap::iterator it = mv_map.find(m);
    if (it != mv_map.end())
      return it->second;
    vertex_ptr v(new vertex(m));
    graph_t::vertex_descriptor d = add_vertex(v, graph);
    mv_map.insert(std::make_pair(m, d));
    return d;
  }

  void
  plasm::impl::connect(cell_ptr from, std::string output, cell_ptr to, std::string input)
  {
    //connect does all sorts of type checking so that connections are always valid.
    tendril_ptr from_port, to_port;
    try {
      from_port = from->outputs[output];
    }
    catch (ecto::NonExistant& e) {
      e << cell_name(from->name())
        << tendril_key(output)
        << which_tendrils("outputs")
        ;
      throw;
    }

    try {
      to_port = to->inputs[input];
    }
    catch (ecto::NonExistant& e) {
      e << cell_name(from->name())
        << tendril_key(input)
        << which_tendrils("inputs")
        ;
      throw;
    }
    //throw if the types are bad... Don't allow erroneous graph construction
    //also this is more local to the error.
    if (!to_port->compatible_type(*from_port))
      {
        std::string s;
        BOOST_THROW_EXCEPTION(TypeMismatch()
                               << from_typename(from_port->type_name())
                               << from_key(output)
                               << from_cell(from->name())
                               << to_typename(to_port->type_name())
                               << to_key(input)
                               << to_cell(to->name()));
      }

    graph_t::vertex_descriptor fromv = insert_module(from), tov = insert_module(to);
    graph::edge_ptr new_edge = make_edge(output, input);

    //assert that the new edge does not violate inputs that are already connected.
    //RULE an input may only have one source.
    graph_t::in_edge_iterator inbegin, inend;
    tie(inbegin, inend) = boost::in_edges(tov, graph);
    while (inbegin != inend)
      {
        graph::edge_ptr e = graph[*inbegin];
        if (e->to_port() == new_edge->to_port())
          {
            BOOST_THROW_EXCEPTION(AlreadyConnected()
                                  << cell_name(to->name())
                                  << tendril_key(e->to_port())
                                  );
          }
        ++inbegin;
      }

    bool added;
    graph_t::edge_descriptor ed;
    tie(ed, added) = boost::add_edge(fromv, tov, new_edge, graph);
    if (!added)
      {
        BOOST_THROW_EXCEPTION(EctoException()
                              << diag_msg("plasm failed to connect cell input/outputs")
                              << from_cell(from->name())
                              << from_key(output)
                              << to_cell(to->name())
                              << to_key(input));
      }
  }

  void
  plasm::impl::disconnect(cell_ptr from, std::string output, cell_ptr to, std::string input)
  {
    graph_t::vertex_descriptor fromv = insert_module(from), tov = insert_module(to);
    boost::remove_edge(fromv, tov, graph);
  }

}

