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

#include <ecto/cell.hpp>
#include <ecto/ecto.hpp>
#include <ecto/edge.hpp>
#include <boost/graph/graphviz.hpp>
#include <ecto/graph/types.hpp>
#include <ecto/serialization/registry.hpp>
#include <ecto/serialization/cell.hpp>
#include <ecto/tendrils.hpp>
#include <ecto/vertex.hpp>
#include <ecto/graph/utilities.hpp>
#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/regex.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/topological_sort.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

namespace ecto
{
  using namespace graph;
#define STRINGIZE(A) #A
  //see http://www.graphviz.org/content/node-shapes for reference.
  const char* table_str = STRINGIZE(
      <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4">
      %s
      <TR>
      %s
      %s
      </TR>
      %s
      %s
      </TABLE>
  );

  const char* input_str = STRINGIZE(
      <TD PORT="i_%s" BGCOLOR="springgreen">%s</TD>
  );

  const char* cell_str = STRINGIZE(<TD ROWSPAN="%d" COLSPAN="%d" BGCOLOR="khaki">%s</TD>
  );

  const char* param_str_1st = STRINGIZE(
      <TD PORT="p_%s" BGCOLOR="lightblue">%s</TD>
  );

  const char* param_str_N = STRINGIZE(
      <TR>
      <TD PORT="p_%s" BGCOLOR="lightblue">%s</TD>
      </TR>
  );

  const char* output_str = STRINGIZE(
      <TD PORT="o_%s" BGCOLOR="indianred1">%s</TD>
  );

  struct vertex_writer
  {
    graph_t* g;

    vertex_writer(graph_t* g_) : g(g_) { }

    std::string
    htmlescape(const std::string& in)
    {
      const boost::regex esc_lt("[<]");
      const std::string rep_lt("&lt;");
      const boost::regex esc_gt("[>]");
      const std::string rep_gt("&gt;");

      std::string htmlescaped_name = in;
      htmlescaped_name = boost::regex_replace(htmlescaped_name, esc_lt, rep_lt, boost::match_default);
      htmlescaped_name = boost::regex_replace(htmlescaped_name, esc_gt, rep_gt, boost::match_default);
      return htmlescaped_name;
    }

    void
    operator()(std::ostream& out, graph_t::vertex_descriptor vd)
    {

      cell_ptr c = (*g)[vd]->cell();
      int n_inputs = c->inputs.size();
      int n_outputs = c->outputs.size();
      int n_params = c->parameters.size();
      std::string htmlescaped_name = htmlescape(c->name());

      std::string inputs;
      BOOST_FOREACH(const tendrils::value_type& x, c->inputs)
          {
            std::string key = x.first;
            if (inputs.empty())
              inputs = "<TR>\n";
            inputs += boost::str(boost::format(input_str) % key % key) + "\n";
          }
      if (!inputs.empty())
        inputs += "</TR>";

      std::string outputs;
      BOOST_FOREACH(const tendrils::value_type& x, c->outputs)
          {
            std::string key = x.first;
            if (outputs.empty())
              outputs = "<TR>\n";
            outputs += boost::str(boost::format(output_str) % key % key) + "\n";
          }
      if (!outputs.empty())
        outputs += "</TR>";

      std::string cellrow = boost::str(
          boost::format(cell_str) % std::max(1, n_params) % int(std::max(1, std::max(n_inputs, n_outputs)))
          % htmlescaped_name);
      std::string p1, pN;
      BOOST_FOREACH(const tendrils::value_type& x, c->parameters)
          {
            std::string key = x.first;
            if (p1.empty())
              p1 = boost::str(boost::format(param_str_1st) % key % key) + "\n";
            else
              pN += boost::str(boost::format(param_str_N) % key % key) + "\n";
          }

      std::string table = boost::str(boost::format(table_str) % inputs % cellrow % p1 % pN % outputs);
      out << "[label=<" << table << ">]";
    }
  };

  struct edge_writer
  {
    graph_t* g;

    edge_writer(graph_t* g_) : g(g_) { }

    void
    operator()(std::ostream& out, graph_t::edge_descriptor ed)
    {
      out << "[headport=\"i_" << (*g)[ed]->to_port()
          << "\" tailport=\"o_" << (*g)[ed]->from_port() << "\"]";
    }
  };

  struct graph_writer
  {
    void
    operator()(std::ostream& out) const
    {
      out << "graph [rankdir=TB, ranksep=1]" << std::endl;
      out << "edge [labelfontsize=8]" << std::endl;
      out << "node [shape=plaintext]" << std::endl;
    }
  };

  plasm::plasm() : impl_(new impl), configured(false) { }

  plasm::~plasm()
  {
  }

  void
  plasm::insert(cell_ptr mod)
  {
    impl_->insert_module(mod);
    configured = false;
  }

  void
  plasm::connect(cell_ptr from, const std::string& output, cell_ptr to, const std::string& input)
  {
    impl_->connect(from, output, to, input);
    configured = false;
  }

  void
  plasm::viz(std::ostream& out) const
  {
    boost::write_graphviz(out, impl_->graph, vertex_writer(&impl_->graph),
                          edge_writer(&impl_->graph), graph_writer());
  }

  std::string
  plasm::viz() const
  {
    std::stringstream ss;
    viz(ss);
    return ss.str();
  }

  void
  plasm::disconnect(cell_ptr from, const std::string& output, cell_ptr to, const std::string& input)
  {
    impl_->disconnect(from, output, to, input);
  }

  graph::graph_t&
  plasm::graph()
  {
    return impl_->graph;
  }

  const graph::graph_t&
  plasm::graph() const
  {
    return impl_->graph;
  }

  std::size_t
  plasm::size() const
  {
    return num_vertices(impl_->graph);
  }

  namespace
  {
    struct get_first
    {
      template<typename T>
      cell_ptr
      operator()(T& t) const
      {
        return t.first;
      }
    };
  }

  std::vector<cell_ptr>
  plasm::cells() const
  {
    std::vector<cell_ptr> c;
    std::transform(impl_->mv_map.begin(), impl_->mv_map.end(), std::back_inserter(c), get_first());
    return c;
  }

  void
  plasm::configure_all()
  {
    if (configured) return;
    /****************************************
     ** Unsorted Configuration - Deprecated
     ****************************************/
    // BOOST_FOREACH(impl::ModuleVertexMap::value_type& x, impl_->mv_map)
    //      {
    //        x.first->configure();
    //      }
    /****************************************
     ** Sorted Configuration - Depth First
     ****************************************/
    std::vector<ecto::graph::graph_t::vertex_descriptor> stack;
    boost::topological_sort(impl_->graph, std::back_inserter(stack));
    std::reverse(stack.begin(), stack.end());
    BOOST_FOREACH(const ecto::graph::graph_t::vertex_descriptor& vd, stack)
    {
      ecto::graph::invoke_configuration(impl_->graph, vd);
    }
    /****************************************
     ** Sorted Configuration - Breadth First
     ****************************************/
    // Note that the default scheduler uses this instead. Bring it in, if needed.

    configured = true;
  }

  void
  plasm::activate_all()
  {
    BOOST_FOREACH(impl::ModuleVertexMap::value_type& x, impl_->mv_map)
        {
          x.first->activate();
        }
  }

  void
  plasm::deactivate_all()
  {
    BOOST_FOREACH(impl::ModuleVertexMap::value_type& x, impl_->mv_map)
        {
          x.first->deactivate();
        }
  }

  void
  plasm::check() const
  {
    graph_t& g(impl_->graph);
    graph_t::vertex_iterator begin, end;
    boost::tie(begin, end) = boost::vertices(g);
    while (begin != end)
    {
      cell_ptr m = g[*begin]->cell();
      std::set<std::string> in_connected, out_connected;

      //verify all required inputs are connected
      graph_t::in_edge_iterator b_in, e_in;
      boost::tie(b_in, e_in) = boost::in_edges(*begin, g);
      while (b_in != e_in)
      {
        edge_ptr in_edge = g[*b_in];
        in_connected.insert(in_edge->to_port());
        ++b_in;
      }

      for (tendrils::const_iterator b_tend = m->inputs.begin(), e_tend = m->inputs.end();
           b_tend != e_tend; ++b_tend)
      {
        if (b_tend->second->required() && in_connected.count(b_tend->first) == 0)
        {
          BOOST_THROW_EXCEPTION(
              except::NotConnected() << except::tendril_key(b_tend->first)
                                     << except::cell_name(m->name()));
        }
      }

      //verify the outputs are connected
      graph_t::out_edge_iterator b_out, e_out;
      boost::tie(b_out, e_out) = boost::out_edges(*begin, g);
      while (b_out != e_out)
      {
        edge_ptr out_edge = g[*b_out];
        out_connected.insert(out_edge->from_port());
        ++b_out;
      }

      for (tendrils::const_iterator b_tend = m->outputs.begin(), e_tend = m->outputs.end();
           b_tend != e_tend; ++b_tend)
      {
        if (b_tend->second->required() && out_connected.count(b_tend->first) == 0)
        {
          BOOST_THROW_EXCEPTION(
              except::NotConnected() << except::tendril_key(b_tend->first)
                                     << except::cell_name(m->name()));
        }
      }

      ++begin;
    }
  }
  void
  plasm::reset_ticks()
  {
    {
      graph_t::vertex_iterator beg, end;
      boost::tie(beg, end) = vertices(impl_->graph);
      while (beg != end)
      {
        vertex_ptr v = impl_->graph[*beg];
        v->reset_tick();
        ++beg;
      }
    }

    {
      graph_t::edge_iterator beg, end;
      boost::tie(beg, end) = edges(impl_->graph);
      while (beg != end)
      {
        edge_ptr e = impl_->graph[*beg];
        e->reset_tick();
        // TODO: We're doing more than resetting ticks in this method. Rename!
        while (! e->empty()) // TODO: Add a clear() method.
          e->pop_front();
        ++beg;
      }
    }
  }

  void
  plasm::save(std::ostream& out) const
  {
    boost::archive::binary_oarchive oa(out);
    oa << *this;
  }

  void
  plasm::load(std::istream& in)
  {
    boost::archive::binary_iarchive ia(in);
    ia >> *this;
  }
}

