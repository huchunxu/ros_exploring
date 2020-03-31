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
#pragma once

#include <Python.h>
#include <ecto/log.hpp>
#include <ecto/forward.hpp>
#include <ecto/profile.hpp>

#include <boost/asio.hpp>
#ifndef BOOST_SIGNALS2_MAX_ARGS  // this is also defined in tendril.hpp (TODO: consolidate)
  #define BOOST_SIGNALS2_MAX_ARGS 3
#endif
#include <boost/signals2/signal.hpp>
#include <boost/thread/mutex.hpp>

#include <ecto/graph/types.hpp>

namespace ecto {

/**
 * TODO: Doc this class.
 * TODO: Need to share io_svc_ instances with other entities (schedulers+)?
 */
class scheduler {
public:
  /** Scheduler states. Values greater than 0 indicate a "running" state.
   */
  enum State {
    /** None of the execute*() methods have been called yet. */
    INIT = 0,
    /** One of the execute*() methods was called and successfully completed
     * the specified number of iterations. */
    RUNNING,
    /** execute() is running, or jobs have been prepared and
     * the specified number of iterations have not been completed. */
    EXECUTING,
    /** stop() was called, but the scheduler has not stopped yet. */
    STOPPING,
    /** stop() completed, or one of the cell::process() calls
     * returned ecto::QUIT and the scheduler is no longer running. */
    FINI = -1,
    /** One of the cell::process() calls returned an error or threw,
     * and the scheduler is no longer running. */
    ERROR = -2
  };

  explicit scheduler(plasm_ptr p);
  ~scheduler();

  /** Synchronously execute plasm for num_iters iterations.
   * @param[in] num_iters The number of iterations to execute the plasm. 0 indicates
   *   that the plasm should be executed until some cell::process() call
   *   returns ecto::QUIT. \attention This call will block indefinately if
   *   num_iters is 0.
   */
  bool execute(unsigned num_iters = 0);
  /** Prepare jobs for execution of a plasm over num_iters iterations.
   * No actual work will be done without calling the run*() methods. This just
   * fills up the io service queues with the jobs required for initialisation
   * and execution of the plasm over the specified number of iterations.
   * @param[in] num_iters The number of iterations to execute the plasm. 0 indicates
   *   that the plasm should be executed until some cell::process() call
   *   returns ecto::QUIT. \attention A call to run() will block indefinately if
   *   num_iters is 0.
   */
  bool prepare_jobs(unsigned num_iters = 0);

  /** Run one job in the calling thread of execution.
   * \note A job is not necessarily (but is usually) a cell::process() call.
   * \attention If using python cells, this method must be called from the main
   *   python thread.
   * @return true indicates that the scheduler is still "running."
   */
  bool run_job();
  /** Run jobs in the calling thread for the specified number of microseconds,
   * or until the io_service is depleted.
   * @param[in] timeout_usec The number of microsecs to run io_service jobs.
   *   \attention Assuming the io_service does not run out of work, this is the
   *   minimum amount of time that will be spent running jobs.
   * @return true indicates that the scheduler is still "running."
   */
  bool run(unsigned timeout_usec);
  /** Run jobs in the calling thread until the io_service is depleted.
   * @return true indicates that the scheduler is still "running."
   */
  bool run();

  /** @return true indicates that the plasm is currently in a running state
   *    (state_ > 0, i.e. RUNNING, EXECUTING, or STOPPING).
   */
  inline bool running() const;
  /** @return true indicates that the plasm is currently in an executing state
   *    ( state_ == EXECUTING).
   */
  inline bool executing() const;

  /** Stop the scheduler, and flush any jobs in the io_service.
   * \note The scheduler will no longer be in the running state.
   * \attention The plasm may be in the middle of the stack when it is stopped,
   *   but successive calls to execute*() start from the beginning.
   */
  void stop();

  /** @return The current graph execution stats. */
  std::string stats() const { return graphstats_.as_string(graph_); }

  /** @return The current scheduler state. */
  inline State state() const;

private:
  inline State state(State);
  void execute_init(unsigned num_iteRelWithDebInfors);
  void execute_iter(unsigned cur_iter, unsigned num_iters,
                    std::size_t stack_idx);
  void execute_fini();
  void interrupt();

  /** Check plasm for correctness, configure it, activate it, then sort it
   * topologically to populate stack_.
   * This method is idempotent.
   */
  void compute_stack();

  plasm_ptr plasm_;
  ecto::graph::graph_t& graph_;

  std::vector<ecto::graph::graph_t::vertex_descriptor> stack_;

  profile::graph_stats_type graphstats_;

  boost::asio::io_service io_svc_;

  mutable boost::mutex mtx_;
  //! Current state of the scheduler.
  State state_;
  //! Current number of "runners" (threads calling a run method).
  std::size_t runners_;

  // sigint handling
  boost::signals2::connection interrupt_connection;
  bool interrupted;
}; // scheduler

template<typename Mutex_T = boost::mutex, typename Count_T = std::size_t>
class ref_count {
public:
  ref_count(Mutex_T & m, Count_T & t)
  : m_(m), t_(t) {
    typename Mutex_T::scoped_lock l(m_);
    ++t_;
  }
  ~ref_count() {
    typename Mutex_T::scoped_lock l(m_);
    --t_;
  }
private:
  Mutex_T & m_;
  Count_T & t_;
};

scheduler::State scheduler::state() const
{
  boost::mutex::scoped_lock l(mtx_);
  return state_;
}

bool scheduler::running() const
{
  boost::mutex::scoped_lock l(mtx_);
  return static_cast<int>(state_) > 0;
}

bool scheduler::executing() const
{
  boost::mutex::scoped_lock l(mtx_);
  return state_ == scheduler::EXECUTING;
}

scheduler::State scheduler::state(State state)
{
  boost::mutex::scoped_lock l(mtx_);
  state_ = state;
  return state_;
}


} // End of namespace ecto.
