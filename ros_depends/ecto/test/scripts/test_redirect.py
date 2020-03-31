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
import sys

fname = "test_redirect.log"

def make(Schedtype):
    print 'Using :', Schedtype
    plasm = ecto.Plasm()

    gen = ecto_test.Generate("Gen", step=1.0, start=0.0)

    printer = ecto_test.Printer("Printy")
    ecto.log_to_file(fname)
    plasm.connect(gen[:] >> printer[:])
    return Schedtype(plasm)

def verify():
    f = open(fname)
    txt = f.read()
    lns = len(txt.splitlines())
    print "txt has", lns, "lines" 
    assert len(txt.splitlines()) >= 5
    
for s in [ecto.Scheduler]:
    print s, "SYNC"
    sched = make(s)
    sched.execute(niter=5)
    ecto.unlog_to_file()
    verify()

    print s, "ASYNC"
    sched2 = make(s)
    sched2.prepare_jobs(niter=5)
    sched2.run()
    ecto.unlog_to_file()
    verify()

