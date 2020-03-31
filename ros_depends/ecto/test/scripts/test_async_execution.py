#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
import ecto
import ecto.ecto_test as ecto_test
from util import fail
import sys, time

print "Hardware concurrency is", ecto.hardware_concurrency()

eps = 0.05

def makeplasm():
    plasm = ecto.Plasm()

    ping = ecto_test.Ping("Ping")
    sleep0 = ecto_test.Sleep("Sleep_0", seconds=0.1)
    sleep1 = ecto_test.Sleep("Sleep_1", seconds=0.1)

    plasm.connect(ping[:] >> sleep0[:], sleep0[:] >> sleep1[:])

    return plasm

def do_test(fn):
    def impl(Sched):
        times = { ecto.Scheduler : 1.0 }
        print "*"*80, "\n", fn.__name__, Sched.__name__
        p = makeplasm()
        s = Sched(p)
        t = times[Sched]
        print "Expecting finish in", t, "seconds"
        fn(s, times[Sched])
    map(impl, [ecto.Scheduler])


def sync(s, ex):
    assert not s.running()
    t = time.time()
    print "starting"
    s.execute(niter=5)
    dur = time.time() - t
    print "done after", dur
    assert dur > ex
    assert dur < (ex + eps)
    assert s.running()


def synctwice(s, ex):
    assert not s.running()
    start_t = time.time()
    print "starting", ex
    s.execute(niter=5)
    dur = time.time() - start_t
    print "HALFWAY:", dur
    assert dur > ex
    assert dur < ex + eps
    assert s.running()
    s.execute(niter=5)
    dur = time.time() - start_t
    print "SECONDTIME:", dur
    assert dur > (ex*2)
    assert dur < ((ex*2) + eps)
    assert s.running()

def ex_async_twice(s, ex):
    assert not s.running()
    s.prepare_jobs(niter=5)
    print "once..."
    assert s.running()
    t = time.time()
    try:
        print "twice..."
        s.prepare_jobs(niter=5)
        fail("that should have thrown")
    except ecto.EctoException, e:
        print "okay, threw"
        print "whee"
    s.run()
    elapsed = time.time() - t
    print "elapsed:", elapsed, "expected:", ex
    assert elapsed > ex
    assert elapsed < (ex + eps)


def ex_async_then_sync_throws(s, ex):
    assert not s.running()
    s.prepare_jobs(niter=5)
    print "once..."
    assert s.running()
    t = time.time()
    try:
        print "twice..."
        s.execute(niter=5)
        fail("that should have thrown")
    except ecto.EctoException, e:
        print "okay, threw"
        print "whee"
    s.run()
    elapsed = time.time() - t
    print "elapsed:", elapsed, "expected:", ex
    assert elapsed > ex
    assert elapsed < (ex + eps)


def wait_on_nothing(s, ex):
    assert not s.running()
    stime = time.time()
    s.run()
    assert not s.running()
    etime = time.time()
    print etime-stime
    assert eps > etime-stime


def running_check(s, ex):
    assert not s.running()
    s.prepare_jobs(niter=5)
    assert s.running()
    s.run()
    assert s.running()


def wait_check(s, ex):
    print __name__, s
    assert not s.running()
    t = time.time()
    s.prepare_jobs(niter=5)
    assert time.time() - t < ex
    s.run()
    print time.time() - t > ex+eps  # we might be multithreaded
    assert s.running()

do_test(wait_on_nothing)
do_test(ex_async_then_sync_throws)
do_test(ex_async_twice)
do_test(sync)
do_test(synctwice)
do_test(running_check)
do_test(wait_check)

# Verify that the multithreaded completes in multiples of two seconds
# from the time stop was called, not the initial start
#def stoppable_multi():
#    hc = ecto.hardware_concurrency()
#    def makeplasm():
#        plasm = ecto.Plasm()
#        ping = ecto_test.Ping("Ping")
#        sleeps = [ecto_test.Sleep("Sleep_0", seconds=1.0/hc)
#                  for x in range(hc)]
#        plasm.connect(ping[:] >> sleeps[0][:])
#        for i in range(1,hc-1):
#            print "i=", i
#            plasm.connect(sleeps[i][:] >> sleeps[i+1][:])
#        return plasm
#
#    p = makeplasm()
#
#    st = ecto.schedulers.Multithreaded(p)
#    st.prepare_jobs()
#    time.sleep(1.3) # wait until we are in steady state
#    start = time.time()
#    st.stop()
#    st.wait()
#    elapsed = time.time() - start
#    print "elapsed multithreaded:", elapsed
#    # we'll be partially through an iteration that has just started
#    print "hc=", hc, "(hc-1.0)/hc=", ((hc-1.0)/hc)
#    assert elapsed >= (hc-1.0)/hc
#    assert elapsed <= (1.0 + eps)
#    st.prepare_jobs()
#    time.sleep(1.0)
#    # this time the start is just before stop is called, not
#    # when execute was called
#    start = time.time()
#    st.stop()
#    st.wait()
#    elapsed = time.time() - start
#    mintime =  (hc-1.0)/hc
#    maxtime = 1.0 + (1.0/hc)
#    print "elapsed multithreade:", elapsed, "expected min:", mintime, \
#          "expected max:", maxtime
#    assert elapsed >= mintime
#    assert elapsed <= maxtime
#
#stoppable_multi()

#
#  Verify that the scheduler completes in multiples of two seconds
#
def stoppable():
    def makeplasm():
        plasm = ecto.Plasm()
        ping = ecto_test.Ping("Ping")
        sleeps = [ecto_test.Sleep("Sleep_0", seconds=0.1)
                  for x in range(20)]
        plasm.connect(ping[:] >> sleeps[0][:])
        for i in range(1,19):
            print "i=", i
            plasm.connect(sleeps[i][:] >> sleeps[i+1][:])
        return plasm

    p = makeplasm()

    st = ecto.Scheduler(p)
    assert not st.running()
    start = time.time()
    st.execute(1)
    elapsed = time.time() - start
    print "elapsed Scheduler:", elapsed
    assert elapsed > 2.0
    assert elapsed < 2.1
    assert st.running()

    start = time.time()
    st.prepare_jobs(1)
    st.run()
    elapsed = time.time() - start
    print "elapsed Scheduler:", elapsed
    assert elapsed > 2.0
    assert elapsed < 2.1

stoppable()

