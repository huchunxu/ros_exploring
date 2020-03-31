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
import ecto_test
import sys


def test_nodelay():
    plasm = ecto.Plasm()

    ping = ecto_test.Ping("Ping")
    metrics = ecto_test.Metrics("Metrics", queue_size=10)
    plasm.connect(ping[:] >> metrics[:])
    
    sched = ecto.Scheduler(plasm)
    sched.execute(niter=10000)
    print "Hz:", metrics.outputs.hz, " Latency in seconds: %f" % metrics.outputs.latency_seconds

    # these are kinda loose
    assert metrics.outputs.hz > 5000
    assert metrics.outputs.latency_seconds < 0.0001

def test_20hz():
    plasm = ecto.Plasm()

    ping = ecto_test.Ping("Ping")
    throttle = ecto_test.Throttle("Throttle", rate=20)
    metrics = ecto_test.Metrics("Metrics", queue_size=10)
    plasm.connect(ping[:] >> throttle[:],
                  throttle[:] >> metrics[:])
    
    sched = ecto.Scheduler(plasm)
    sched.execute(niter=100)
    print "Hz:", metrics.outputs.hz, " Latency in seconds: %f" % metrics.outputs.latency_seconds

    # these are kinda loose
    assert 19 < metrics.outputs.hz < 21
    assert 0.04 < metrics.outputs.latency_seconds < 0.06

def makeplasm(n_nodes):
    plasm = ecto.Plasm()
    
    ping = ecto_test.Ping("Ping")
    throttle = ecto_test.Sleep("Sleep_0", seconds=1.0/n_nodes)

    plasm.connect(ping[:] >> throttle[:])

    for j in range(n_nodes-1): # one has already been added
        throttle_next = ecto_test.Sleep("Sleep_%u" % (j+1), seconds=1.0/n_nodes)
        plasm.connect(throttle, "out", throttle_next, "in")
        throttle = throttle_next

    metrics = ecto_test.Metrics("Metrics", queue_size=4)
    plasm.connect(throttle[:] >> metrics[:])
    
#    o = open('graph.dot', 'w')
#    print >>o, plasm.viz()
#    o.close()
#    print "\n", plasm.viz(), "\n"

    return (plasm, metrics)

def test_st(niter, n_nodes):

    (plasm, metrics) = makeplasm(n_nodes)

    #sched = ecto.Scheduler(plasm)
    #sched.execute(niter)

    sched = ecto.Scheduler(plasm)
    sched.execute(niter)
    print "Hz:", metrics.outputs.hz, " Latency in seconds:", metrics.outputs.latency_seconds
    assert 0.95 < metrics.outputs.hz < 1.05
    assert 0.95 < metrics.outputs.latency_seconds < 1.05

#
# It is hard to test the middle cases, i.e.  if you have one thread
# per node, things should run at n_nodes hz and 1 second latency but
# if there are less than that, things are somewhere in the middle.
# Also your latency tends to be worse as you have to wait for the
# graph to "fill up"
#
def test_tp(niter, n_nodes):

    (plasm, metrics) = makeplasm(n_nodes)

    sched = ecto.Scheduler(plasm)
    sched.execute(niter=niter)
    print "Hz:", metrics.outputs.hz, " Latency in seconds:", metrics.outputs.latency_seconds
    assert n_nodes * 0.95 < metrics.outputs.hz < n_nodes * 1.05
    assert 0.9 < metrics.outputs.latency_seconds < 1.1



test_nodelay()
test_20hz()
test_st(5, 5)
test_st(5, 12)

test_tp(20, 15)
test_tp(20, 10)
test_tp(20, 5)

