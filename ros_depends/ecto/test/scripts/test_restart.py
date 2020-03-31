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

import sys, ecto, ecto_test

def makeplasm(N):
    plasm = ecto.Plasm()
    gen = ecto_test.Generate(start=1, step=1)
    quitter = ecto_test.QuitAfter(N=N)
    plasm.connect(gen[:] >> quitter[:])

    return (quitter, gen, plasm)

# def do_one_st(N, j):
#     #print "multithreaded test w/ quit after", N
#     (gen, plasm) = makeplasm(N)
#
#     sched = ecto.Scheduler(plasm)
#     for i in range(j):
#         sched.execute(niter=N+10)
#         # print "="*70
#
#     #print "singlethreaded: actual out:", gen.outputs.out, " N:", N
#     assert (j*N - 1.0) == gen.outputs.out
#     #print "\n" * 5
#
# do_one_st(1,1)
# do_one_st(1,2)
# do_one_st(2,1)
# do_one_st(2,2)
# for N in range(1, 100, 10):
#     for loops in range(2, 10, 2):
#         do_one_st(N, loops)


def do_one_impl(SchedType, countto, nthreads, niter):
    print "*"*80, "\n", SchedType, "test w/ quit after", countto, " nthreads=", nthreads, "niter=", niter
    (quitter, gen, plasm) = makeplasm(countto)

    sched = SchedType(plasm)

    quitout = countto
    for j in range(niter):
        sched.execute(niter=countto+100)
        print sched.stats()
        print "j:", j
        print "niter:", countto+100
        print "countto:", countto
        print "nthreads:", nthreads
        print "quitter.out: ", quitter.outputs.out
        print "gen.out: ", gen.outputs.out
        print "quitout:", quitout
        assert quitter.outputs.out == quitout
        quitout += countto + 1 # because the gen will have fired an unused value
            # assert gen.outputs.out == countto + dist

def do_one(countto, nthreads, niter):
    for S in [ecto.Scheduler]:
        do_one_impl(S, countto, nthreads, niter)

do_one(1, 1, 1)
do_one(1, 2, 1)
do_one(2, 1, 2)
do_one(2, 8, 2)

for i in range(1, 200, 13):
    for nthreads in range(2, 10, 2):
        for niter in range(2, 10, 2):
            do_one(i, nthreads, niter)





