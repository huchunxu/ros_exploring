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

#if !defined(_WIN32)
#include <unistd.h>
#endif
#include <boost/format.hpp>
#include <ecto/graph/types.hpp>

namespace pt = boost::posix_time;

namespace ecto {
namespace profile {

#if defined(__i386__)

unsigned long read_tsc(void)
{
  unsigned long tsc;
  asm(".byte 0x0f, 0x31" : "=A" (tsc));
  return( tsc );
}

#elif (defined(__amd64__) || defined(__x86_64__))

unsigned long read_tsc(void)
{
  unsigned long lo, hi;
  asm( "rdtsc" : "=a" (lo), "=d" (hi) );
  return( lo | (hi << 32) );
}

#elif (defined(__powerpc__) || defined(__ppc__))

unsigned long read_tsc(void)
{
  unsigned long tbl, tbu0, tbu1;

  do {
      asm( "mftbu %0" : "=r" (tbu0) );
      asm( "mftb  %0" : "=r" (tbl ) );
      asm( "mftbu %0" : "=r" (tbu1) );
  } while( tbu0 != tbu1 );

  return( tbl );
}

#elif defined(__sparc__)

unsigned long read_tsc(void)
{
  unsigned long tick;
  asm( ".byte 0x83, 0x41, 0x00, 0x00" );
  asm( "mov   %%g1, %0" : "=r" (tick) );
  return( tick );
}

#elif defined(__alpha__)

unsigned long read_tsc(void)
{
  unsigned long cc;
  asm( "rpcc %0" : "=r" (cc) );
  return( cc & 0xFFFFFFFF );
}

#elif defined(__ia64__)

unsigned long read_tsc(void)
{
  unsigned long itc;
  asm( "mov %0 = ar.itc" : "=r" (itc) );
  return( itc );
}

#else

unsigned long read_tsc(void)
{
 //todo FIXME.
  return 0;
}

#endif

stats_type::stats_type()
  : ncalls(0), total_ticks(0)
{ }

namespace {
/** This is the system clock refresh rate, not to be confused w/
 * the value returned by read_tsc. */
double clock_ticks_per_second()
{
#if !defined(_WIN32)
  return static_cast<double>(sysconf(_SC_CLK_TCK));
#else
  return 1.;
#endif
}
}

double stats_type::elapsed_time()
{
#if !defined(_WIN32)
  return total_ticks / clock_ticks_per_second();
#else
  return 1.0;
#endif
}

double stats_type::frequency()
{
  return ncalls/elapsed_time();
}

std::string graph_stats_type::as_string(graph::graph_t& g) const
{
  std::ostringstream oss;

  const std::string hline = "------------------------------------------------------------------------------\n";
  oss << hline;
  oss << str(boost::format("* %-25s   %7s %12s %12s %8s\n")
             % "Cell Name"
             % "Calls"
             % "Time (ms)"
             % "Hz(observed)"
             % "load (%)");

  double total_percentage = 0.0;
  const unsigned total_microseconds = cumulative_time.total_microseconds();
  const unsigned total_milliseconds = cumulative_time.total_milliseconds();
  const double ticks_per_sec = (! total_milliseconds) ?  0. :
    (cumulative_ticks / (total_milliseconds / 1000.));

  graph::graph_t::vertex_iterator begin, end;
  for (boost::tie(begin, end) = vertices(g); begin != end; ++begin) {
    const graph::vertex_ptr vp = g[*begin];
    const double this_ticks = static_cast<double>(vp->stats().total_ticks);
    const double this_time = (! total_milliseconds) ? 0. :
      (this_ticks / (ticks_per_sec / 1000.));
    const double this_percentage = (! cumulative_ticks) ? 0. :
      this_ticks / cumulative_ticks;
    total_percentage += this_percentage;
    assert(total_percentage <= 1.);
    const double hz = (! total_microseconds) ? 0. :
                      (static_cast<double>(vp->stats().ncalls) * 1e+06)
                     / total_microseconds;
    oss << str(boost::format("* %-25s   %7u %12.2f %12.2f %8.2lf")
               % vp->cell()->name()
               % vp->stats().ncalls
               % this_time
               % hz
               % (this_percentage * 100)
              )
        << "\n";
  }
  const double sched_percentage = 1-total_percentage;
  oss << str(boost::format("* %-25s   %7s %12s %12s %8.2lf\n")
             % "Scheduler overhead, etc." % "NA" % "NA" % "NA"
             % (sched_percentage * 100)
            )
      << "\n";

  oss << hline
      << "cpu ticks:        " << cumulative_ticks
      << " (@ "
      << (ticks_per_sec / 1e+9) << " GHz)\n"
      << "elapsed time:     " << cumulative_time
      << " (timer resolution: " << 1.0 / cumulative_time.ticks_per_second() << " second)\n"
    ;
  return oss.str();
}

} // End of namespace profile.
} // End of namespace ecto.

