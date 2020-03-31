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
import sys, time
import random

def makeplasm():
    plasm = ecto.Plasm()

    ping = ecto_test.Ping("Ping")
    sleep0 = ecto_test.SleepPyObjectAbuser(list_o_sleeps=[random.random() * 0.1 for i in range(1,10)])
    sleep1 = ecto_test.SleepPyObjectAbuser(list_o_sleeps=[random.random() * 0.1 for i in range(1,10)])

    plasm.connect(ping[:] >> sleep0[:],
                  sleep0[:] >> sleep1[:])

    return plasm

def async(s):
    print "s.prepare_jobs"
    s.prepare_jobs(niter=5)
    assert s.running()

def sync(s):
    print "s.execute"
    s.execute(niter=5)
    assert s.running()

def nada(s):
    print "nada"

def waitonly(s):
    print "waitonly"
    s.run()
    assert s.running()
    print "wait DONE"

def tpool(Scheduler, go, afterwards, sleepdur=0.1):
    print "*"*80
    print Scheduler, go, afterwards, sleepdur
    p = makeplasm()
    s = Scheduler(p)
    go(s)
    #this is where it fails.
    #bp::stl_input_iterator<double> begin(list_o_sleeps),end;
    print "time.sleep(", sleepdur, ")"
    stime = time.time()
    time.sleep(sleepdur)
    afterwards(s)
    etime = time.time()
    print "elapsed", etime-stime

def doemall(Sched):
    tpool(Sched, sync, nada)
    tpool(Sched, sync, waitonly)
    tpool(Sched, async, waitonly)
    tpool(Sched, async, nada)

doemall(ecto.Scheduler)

