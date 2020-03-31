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
#include <stdint.h>//int64_t

#include <Python.h>
#include <ecto/forward.hpp>
#include <ecto/util.hpp>
#include <ecto/log.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace ecto {
namespace profile {

unsigned long read_tsc();

struct graph_stats_type
{
  graph_stats_type() : cumulative_time(), cumulative_ticks(0) {}
  boost::posix_time::time_duration cumulative_time;
  uint64_t cumulative_ticks;

  std::string as_string(graph::graph_t& g) const;
};

struct graphstats_collector
{
  graph_stats_type& gs_;
  const boost::posix_time::ptime start_time;
  const unsigned long start_tick;
  graphstats_collector(graph_stats_type& gs)
    : gs_(gs)
    , start_time(boost::posix_time::microsec_clock::universal_time())
    , start_tick(profile::read_tsc())
  {
  }

  ~graphstats_collector()
  {
    const unsigned long stop_tick = profile::read_tsc();
    const boost::posix_time::ptime stop_time =
      boost::posix_time::microsec_clock::universal_time();
    gs_.cumulative_time  += stop_time - start_time;
    gs_.cumulative_ticks += stop_tick - start_tick;
  }
};

struct ECTO_EXPORT stats_type
{
  stats_type();
  unsigned ncalls;
  uint64_t total_ticks;
  bool on;

  double elapsed_time();
  double frequency();
};

struct ECTO_EXPORT stats_collector
{
  const unsigned long start;
  stats_type& stats;
  const std::string& instancename;

  stats_collector(const std::string& n, stats_type& stats)
    : start(read_tsc()), stats(stats), instancename(n)
  {
    ++stats.ncalls;
    stats.on = true;
    //ECTO_LOG_PROCESS(instancename, start, stats.ncalls, 1);
  }

  ~stats_collector() {
    const unsigned long tsc = read_tsc();
    //ECTO_LOG_PROCESS(instancename, tsc, stats.ncalls, 0);
    stats.total_ticks += (tsc - start);
    stats.on = false;
  }
};

} // End of namespace profile.
} // End of namespace ecto.
