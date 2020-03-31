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
import ecto, sys
import ecto.ecto_test as ecto_test
from ecto.test import test

@test
#def test_strand_basic_semantics():
#    s = ecto.Strand()
#    print "s.id =", s.id
#    orig_id = s.id
#
#    c = ecto_test.DontCallMeFromTwoThreads("CRASHY", strand=s)
#    c2 = ecto_test.DontCallMeFromTwoThreads("CRASHY2", strand=s)
#    c3 = ecto_test.DontCallMeFromTwoThreads("CRASHY3", strand=s)
#    p = ecto.Plasm()
#    gen = ecto_test.Generate("GENERATE", step=1.0, start=1.0)
#    p.connect(gen[:] >> c[:])
#    p.connect(c[:] >> c2[:])
#    p.connect(c2[:] >> c3[:])
#    sched = ecto.schedulers.Multithreaded(p)
#    sched.execute(10)

@test
def test_user_defined_strands(nlevels, SchedType, execfn, expect):
    s1 = ecto.Strand()
    s2 = s1
    s3 = ecto.Strand()

    print "s1.id ==", s1.id
    print "s2.id ==", s2.id
    print "s3.id ==", s3.id
    assert s1.id == s2.id
    assert s3.id != s2.id
    assert s3.id != s1.id

    plasm = ecto.Plasm()
    # plasm.movie_out("strands_%03d.dot")

    gen = ecto_test.Generate("GENERATE", step=1.0, start=1.0)
    noncurr = ecto_test.DontCallMeFromTwoThreads("ALPHA", strand=s1)
    plasm.connect(gen[:] >> noncurr[:])

    for k in range(nlevels):
        n = ecto_test.DontCallMeFromTwoThreads("BETA_%d" % k, strand=s2)
        plasm.connect(noncurr[:] >> n[:])
        noncurr = n

    printer = ecto_test.Printer("PRINTER")
    plasm.connect(noncurr[:] >> printer[:])

    sched = SchedType(plasm)
    print "sched=", sched
    execfn(sched)

    result = noncurr.outputs.out
    print "result=", result, "expect=", expect
    assert(result == expect)
#    execfn(sched)
#    result = noncurr.outputs.out
#    print "result=", result

@test
def test_implicit_strands(nlevels, SchedType, execfn, expect):
    plasm = ecto.Plasm()

    gen = ecto_test.Generate(step=1.0, start=1.0)
    noncurr = ecto_test.CantCallMeFromTwoThreads()
    plasm.connect(gen, "out", noncurr, "in")

    for k in range(nlevels):
        next = ecto_test.CantCallMeFromTwoThreads()
        plasm.connect(noncurr, "out", next, "in")
        noncurr = next

    printer = ecto_test.Printer()
    plasm.connect(noncurr, "out", printer, "in")

    sched = SchedType(plasm)
    print "sched=", sched
    execfn(sched)

    result = noncurr.outputs.out
    print "result=", result
    assert(result == expect)

@test
def shouldfail():
    plasm = ecto.Plasm()

    gen = ecto_test.Generate(step=1.0, start=1.0)
    nc1 = ecto_test.DontCallMeFromTwoThreads()
    plasm.connect(gen, "out", nc1, "in")

    nc2 = ecto_test.DontCallMeFromTwoThreads()
    plasm.connect(nc1, "out", nc2, "in")

    printer = ecto_test.Printer()
    plasm.connect(nc2, "out", printer, "in")

    sched = ecto.Scheduler(plasm)
    try:
        print "about to execute... this should throw"
        sched.execute(niter=4)
        util.fail()
    except RuntimeError, e:
        print "good, python caught error", e

    sched.stop()
    sched.wait()

#test_strand_basic_semantics()

#shouldfail()
#print "shouldfail passed"

test_implicit_strands(4, ecto.Scheduler, lambda s: s.execute(niter=4), expect=4.0)

test_user_defined_strands(4, ecto.Scheduler, lambda s: s.execute(niter=16), expect=16.0)



