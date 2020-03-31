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

eps = 0.1

def makeplasm():
    plasm = ecto.Plasm()

    ping = ecto_test.Ping("Ping")
    startstop = ecto_test.StartStopCounter()

    plasm.connect(ping[:] >> startstop[:])

    return (plasm, startstop)

def do_test(fn):
    def impl(Sched):
        print "*"*80, "\n", fn.__name__, Sched.__name__
        (p, ss) = makeplasm()
        s = Sched(p)
        fn(s, ss)
    map(impl, [ecto.Scheduler])


def synctwice(s, ss):

    print "*"*80
    print "\n"*5
    assert ss.outputs.nstart == 0
    assert ss.outputs.nstop == 0
    assert ss.outputs.nconfigure == 0
    assert ss.outputs.nprocess == 0

    # Test synchronous execute with no stops.
    iters = 3
    for i in range(iters):
        s.execute(niter=5)

        print "NSTART=", ss.outputs.nstart
        assert ss.outputs.nstart == 1
        print "NSTOP=", ss.outputs.nstop
        assert ss.outputs.nstop == 0
        assert ss.outputs.nconfigure == 1
        print "NPROCESS=", ss.outputs.nprocess
        assert ss.outputs.nprocess == 5*(i+1)

    # Test asynchronous execute with no stops.
    for i in range(iters):
        s.prepare_jobs(niter=5)
        s.run()

        print "NSTART=", ss.outputs.nstart
        assert ss.outputs.nstart == 1
        print "NSTOP=", ss.outputs.nstop
        assert ss.outputs.nstop == 0
        assert ss.outputs.nconfigure == 1
        print "NPROCESS=", ss.outputs.nprocess
        assert ss.outputs.nprocess == 5*iters + 5*(i+1)

    # Test synchronous execute with stops
    for i in range(iters):
        s.stop()
        s.execute(niter=5)

        print "NSTART=", ss.outputs.nstart
        assert ss.outputs.nstart == 1 + i + 1
        print "NSTOP=", ss.outputs.nstop
        assert ss.outputs.nstop == i + 1
        assert ss.outputs.nconfigure == 1
        print "NPROCESS=", ss.outputs.nprocess
        assert ss.outputs.nprocess == 2*5*iters + 5*(i+1)

    # Test asynchronous execute with stops
    for i in range(iters):
        s.stop()
        s.prepare_jobs(niter=5)
        s.run()

        print "NSTART=", ss.outputs.nstart
        assert ss.outputs.nstart == iters + 1 + i + 1
        print "NSTOP=", ss.outputs.nstop
        assert ss.outputs.nstop == iters + i + 1
        assert ss.outputs.nconfigure == 1
        print "NPROCESS=", ss.outputs.nprocess
        assert ss.outputs.nprocess == 3*5*iters + 5*(i+1)

    # Test partial asynchronous execution
    s.prepare_jobs()
    for i in range(2): # 2 cell::process() jobs + no execute_init() jobs.
        s.run_job() # Make sure params, etc are initialized, and process() is called once on each cell.
    s.stop()
    print s.stats(), "\n"*5
    print "NSTART=", ss.outputs.nstart
    assert ss.outputs.nstart == 2*iters + 1
    print "NSTOP=", ss.outputs.nstop
    assert ss.outputs.nstop == 2*iters + 1
    assert ss.outputs.nconfigure == 1
    print "NPROCESS=", ss.outputs.nprocess
    assert ss.outputs.nprocess == 4*5*iters + 1

    s.execute(niter=5)

    #print "NSTART=", ss.outputs.nstart
    #assert ss.outputs.nstart == 2
    #print "NSTOP=", ss.outputs.nstop
    #assert ss.outputs.nstop == 1
    #assert ss.outputs.nconfigure == 1
    #print "NPROCESS=", ss.outputs.nprocess
    #assert ss.outputs.nprocess == 10

for j in range(ecto.test.iterations):
    do_test(synctwice)

def things_not_too_slow(s, ss):
    s.prepare_jobs()
    s.stop()
    print s.stats()
    s.prepare_jobs()
    s.stop()
    print s.stats()

do_test(things_not_too_slow)
