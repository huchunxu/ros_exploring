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

#include <ecto/scheduler.hpp>

#include <ecto/cell.hpp>
#include <ecto/graph/utilities.hpp>
#include <ecto/log.hpp>
#include <ecto/plasm.hpp>
#include <ecto/vertex.hpp>

#include <boost/date_time/microsec_time_clock.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/thread.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/scoped_ptr.hpp>

#include <signal.h>

namespace ecto {
using namespace ecto::except;
using ecto::graph::graph_t;
using boost::scoped_ptr;
using boost::thread;
using boost::mutex;

namespace {
  boost::signals2::signal<void(void)> SINGLE_THREADED_SIGINT_SIGNAL;
  void sigint_static_thunk(int)
  {
    std::cerr << "*** SIGINT received, stopping graph execution.\n"
              << "*** If you are stuck here, you may need to hit ^C again\n"
              << "*** when back in the interpreter thread.\n"
              << "*** or Ctrl-\\ (backslash) for a hard stop.\n" << std::endl;
    SINGLE_THREADED_SIGINT_SIGNAL();
  }
} // End of anonymous namespace.

scheduler::scheduler(plasm_ptr p)
: plasm_(p)
, graph_(p->graph())
, io_svc_()
, state_(INIT)
, runners_(0)
, interrupt_connection(SINGLE_THREADED_SIGINT_SIGNAL.connect(boost::bind(&scheduler::interrupt, this)))
, interrupted(false)
{
  assert(plasm_);
#if !defined(_WIN32)
  // TODO (JTF): Move this somewhere else, and use sigaction(2) instead.
  signal(SIGINT, &sigint_static_thunk);
#endif
}

scheduler::~scheduler()
{
  interrupt_connection.disconnect();
  stop();
}

bool scheduler::execute(unsigned num_iters)
{
  prepare_jobs(num_iters);
  run();
  return (state_ > 0); // NOT thread-safe!
}

bool scheduler::prepare_jobs(unsigned num_iters)
{
  { // BEGIN mtx_ scope.
    mutex::scoped_lock l(mtx_);
    if (EXECUTING == state_)
      BOOST_THROW_EXCEPTION(EctoException()
                            << diag_msg("Scheduler already executing"));

    // Make sure the io_service is ready to go.
    io_svc_.reset();
    if (state_ != RUNNING) {
      io_svc_.post(boost::bind(& scheduler::execute_init, this, num_iters));
    } else {
      io_svc_.post(boost::bind(& scheduler::execute_iter, this,
        0 /* cur_iter */, num_iters, 0 /* stack_idx */));
    }
    state_ = EXECUTING; // Make sure no one else can start an execution.
  } // END mtx_ scope.

  return (state_ > 0); // NOT thread-safe!
}

bool scheduler::run_job()
{
  ref_count<> c(mtx_, runners_);
  profile::graphstats_collector gs(graphstats_); // TODO: NOT thread-safe!
  io_svc_.run_one();
  return (state_ > 0); // NOT thread-safe!
}

bool scheduler::run(unsigned timeout_usec)
{
  ref_count<> c(mtx_, runners_);
  profile::graphstats_collector gs(graphstats_); // TODO: NOT thread-safe!
  using namespace boost::posix_time;
  ptime quit = microsec_clock::universal_time() + microseconds(timeout_usec);
  std::size_t n = 0;
  {
    // by default let procesing be free of the gil
    ECTO_SCOPED_GILRELEASE();
    do {
      n = io_svc_.run_one(); // Sit and spin.
    } while (n && microsec_clock::universal_time() < quit);
  }
  return (state_ > 0); // NOT thread-safe!
}

bool scheduler::run()
{
  ref_count<> c(mtx_, runners_);
  profile::graphstats_collector gs(graphstats_); // TODO: NOT thread-safe!
  {
    // by default let procesing be free of the gil
    ECTO_SCOPED_GILRELEASE();
    io_svc_.run();
  }
  return (state_ > 0); // NOT thread-safe!
}

void scheduler::interrupt() {
  interrupted = true;
}

void scheduler::stop()
{
  //std::cerr << this << " scheduler::stop()\n";
  if (! running()) return;
  state(STOPPING);
  run(); // Flush all jobs.
  io_svc_.stop();
  //while (! io_svc_.stopped()) {} // TODO: Need updated version of boost!
  while (true) {
    boost::mutex::scoped_lock l(mtx_);
    if (! runners_) break; // TODO: Use condition variable?
  }
  execute_fini();
  assert(state() == FINI);
  assert(! running());
}

void scheduler::execute_init(unsigned num_iters)
{
  //std::cerr << this << " scheduler::execute_init(" << num_iters << "): STATE="
  //          << state() << "\n";
  if (state() == STOPPING) return; // Flush all jobs from the io_service.
  assert(state() == EXECUTING);

  compute_stack();
  plasm_->reset_ticks();

  // TODO: Should plasm have a reset_ method for strands?
  // Reset cell strands and invoke their start() method.
  for (std::size_t j=0; j<stack_.size(); ++j) {
    cell::ptr c = graph_[stack_[j]]->cell();
    if (! c) continue;
    if (c->strand_)
      c->strand_->reset();
    c->start();
  }
  io_svc_.post(boost::bind(& scheduler::execute_iter, this,
                           0 /* cur_iter */, num_iters, 0 /* stack_idx */));
}

void scheduler::execute_iter(unsigned cur_iter, unsigned num_iters,
                             std::size_t stack_idx)
{
  //std::cerr << this << " scheduler::execute_iter(" << cur_iter << ","
  //          << num_iters << "," << stack_idx << "): STATE=" << state() << " stack_.size()=" << stack_.size() << "\n";
  if (state() == STOPPING) return; // Flush all jobs from the io_service.

  assert(stack_idx < stack_.size());
  assert(state() == EXECUTING);

  int retval = ecto::QUIT;
  try {
    retval = ecto::graph::invoke_process(graph_, stack_[stack_idx]);
    if (interrupted) {
      retval = ecto::QUIT;
      interrupted = false;
    }
  } catch (const boost::thread_interrupted &) {
    std::cout << "Interrupted\n";
  } catch (...) {
    ECTO_LOG_DEBUG("%s", "STOPPING... somebody done threw something.");
    state(ERROR);
    throw; // Propagate to the calling thread.
  }

  switch (retval) {
    case ecto::CONTINUE:
      // unimplemented (move to default) -> https://github.com/plasmodic/ecto/issues/251

    case ecto::BREAK:
    case ecto::OK:
    {
      ++stack_idx;
      if (stack_.size() <= stack_idx || retval == ecto::BREAK) {
        stack_idx = 0;
        ++cur_iter;

        // Made it through the stack. Do it again?
        if (num_iters && cur_iter >= num_iters) {
          // No longer executing, but still "running".
          state(RUNNING);
          return;
        }
      }
      break; // continue execution in this method.
    }

    case ecto::DO_OVER:
      break; // Reschedule this cell i.e. don't bump the stack index.

    default:
    {
      // Don't schedule any more cells, just finalize and quit.
      io_svc_.post(boost::bind(& scheduler::execute_fini, this));
      return;
    }
  }

  io_svc_.post(boost::bind(& scheduler::execute_iter, this,
                           cur_iter, num_iters, stack_idx));
}

void scheduler::execute_fini()
{
  //std::cerr << this << " execute_fini(): STATE=" << state() << "\n";
  assert(running());

  for (std::size_t j=0; j<stack_.size(); ++j) {
    cell::ptr c = graph_[stack_[j]]->cell();
    if (! c) continue;
    c->stop();
  }
  state(FINI);
}

void scheduler::compute_stack()
{
  if (! stack_.empty()) // stack_ will be empty if it needs to be computed.
    return;

  // Check the plasm for correctness, and make sure it is configured/activated.
  plasm_->check();
  plasm_->configure_all();
  plasm_->activate_all();
  ECTO_LOG_DEBUG("graph size = %u", num_vertices(graph_));

#define BREADTHFIRST
#ifdef BREADTHFIRST
// TODO: Doc why we want a "BFS" topological sort here.

  graph_t::vertex_iterator vit, vend;
  // NOTE: We only need to reset the vertex ticks here, and
  // plasm::reset_ticks() also resets the edge ticks.
  boost::tie(vit, vend) = vertices(graph_);
  for (; vit != vend; ++vit)
    graph_[*vit]->reset_tick();

  const std::size_t NUM_VERTICES = num_vertices(graph_);
  for (size_t n = 0; n < NUM_VERTICES; ++n) {
    boost::tie(vit, vend) = vertices(graph_);
    for (; vit != vend; ++vit) {
      // NOTE: tick is incremented on visit
      const graph::vertex_ptr vp = graph_[*vit];
      if (vp->tick() != 0)
        continue; // Already visited this vertex.

      graph_t::in_edge_iterator iebegin, ieend;
      boost::tie(iebegin, ieend) = in_edges(*vit, graph_);
      bool all_ins_visited = true;
      for (; iebegin != ieend; ++iebegin) {
        const graph::vertex_ptr in_vp = graph_[source(*iebegin, graph_)];
        if (in_vp->tick() == 0)
          all_ins_visited = false;
      }
      if (all_ins_visited) {
        vp->inc_tick();
        stack_.push_back(*vit);

        // Check for cycles.
        graph_t::out_edge_iterator oebegin, oeend;
        boost::tie(oebegin, oeend) = out_edges(*vit, graph_);
        for (; oebegin != oeend; ++oebegin) {
          const graph::vertex_ptr out_vp = graph_[target(*oebegin, graph_)];
          if (out_vp->tick()) // Back edge!
            BOOST_THROW_EXCEPTION(EctoException() << diag_msg("Plasm NOT a DAG!"));
        }
      }
    }
  }

  // NOTE: We should insert a vertex each iteration.
  if (NUM_VERTICES != stack_.size())
    BOOST_THROW_EXCEPTION(EctoException() << diag_msg("Plasm NOT a DAG!"));

#else // dfs, which is what boost::topological_sort does
  boost::topological_sort(graph_, std::back_inserter(stack_));
  std::reverse(stack_.begin(), stack_.end());
#endif

  assert(! stack_.empty());
}

#if 0
void scheduler::stop()
{
  // TODO (JTF): This isn't really safe, and we shouldn't really need it w/
  // io_svc_ controlling execution.
  graph_[stack_[0]]->stop_requested(true);
}
#endif

} // End of namespace ecto.
