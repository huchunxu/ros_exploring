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

import sys, ecto
import ecto.ecto_test as ecto_test

def makeplasm(N):
    plasm = ecto.Plasm()
    gen = ecto_test.Generate(start=1, step=1)
    thrower = ecto_test.ThrowAfter(N=N)
    mult = ecto_test.Multiply()

    plasm.connect(gen[:] >> thrower[:],
                  thrower[:] >> mult[:])

    return plasm

def do_one_impl(Sched, nthreads, niter):
    print "\n"*5, "*"*80
    print Sched, nthreads, niter
    p = makeplasm(niter)

    s = Sched(p)
    try:
        s.execute(niter=niter+10)
        assert False, "that should have thrown"
    except ecto.EctoException, e:
        print "okay:", e


def do_one(nthreads, niter):
    for S in [ecto.Scheduler]:
        do_one_impl(S, nthreads, niter)

for j in range(ecto.test.iterations):
    for q in range(10):
        for nthreads in range(1, 10):
            for niter in range(1, 100):
                do_one(nthreads, niter)






