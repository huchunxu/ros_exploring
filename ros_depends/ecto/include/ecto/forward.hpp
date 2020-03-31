/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include <boost/shared_ptr.hpp>

namespace ecto {
  class tendril;
  typedef boost::shared_ptr<tendril> tendril_ptr;
  typedef boost::shared_ptr<const tendril> tendril_cptr;

  class tendrils;
  typedef boost::shared_ptr<tendrils> tendrils_ptr;
  typedef boost::shared_ptr<const tendrils> tendrils_cptr;

  struct cell;
  typedef boost::shared_ptr<cell> cell_ptr;
  typedef boost::shared_ptr<const cell> cell_cptr;

  struct plasm;
  typedef boost::shared_ptr<plasm> plasm_ptr;
  typedef boost::shared_ptr<const plasm> plasm_cptr;

  struct strand;
  typedef boost::shared_ptr<strand> strand_ptr;
  typedef boost::shared_ptr<const strand> strand_cptr;

  template <typename T> struct cell_;

  class scheduler;

  namespace graph {
    struct edge;
    typedef boost::shared_ptr<edge> edge_ptr;
    typedef boost::shared_ptr<const edge> edge_cptr;

    struct vertex;
    typedef boost::shared_ptr<vertex> vertex_ptr;
    typedef boost::shared_ptr<const vertex> vertex_cptr;

    struct graph_t;
  } // End of namespace graph.

} // End of namespace ecto.


