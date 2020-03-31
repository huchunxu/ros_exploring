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

def build_addergraph(nlevels):
    
    plasm = ecto.Plasm()

    prevlevel = [ecto_test.Add("Adder 0_%u" % x) for x in range(2**(nlevels-1))]
    for adder in prevlevel:
        gen0 = ecto_test.Generate("Generator", step=1.0, start=1.0)
        gen1 = ecto_test.Generate("Generator", step=1.0, start=1.0)
        conn1 = gen0["out"] >> adder["left"]
        conn2 = gen1["out"] >> adder["right"]
        # print "conn1=", conn1
        plasm.connect(
            conn1,
            conn2
            )

    # print "prev has", len(prevlevel)
        
    for k in range(nlevels-2, -1, -1):
        # print "****** k=", k, " ***********"
        thislevel = [ecto_test.Add("Adder %u_%u" % (k, x)) for x in range(2**k)]
        # print "prevlevel=", prevlevel
        # print "thislevel=", thislevel
        index = 0
        # print "for...", range(2**k)
        for r in range(2**k):
            # print "prev[%u] => cur[%u]" % (index, r)
            conn = prevlevel[index]["out"] >> thislevel[r]["left"]
            # print "conn=", conn
            plasm.connect(conn)
            index += 1
            # print "prev[%u] => cur[%u]" % (index, r)
            conn2 = prevlevel[index]["out"]>>thislevel[r]["right"]
            # print "conn2=", conn2
            plasm.connect(conn2)
            index += 1
        prevlevel = thislevel

    assert len(prevlevel) == 1
    final_adder = prevlevel[0]
    printer = ecto_test.Printer("printy!")
    #plasm.connect(final_adder, "out", printer, "in")

    return (plasm, final_adder)

def test_plasm_impl(sched_type, nlevels, nthreads, niter):
    (plasm, outnode) = build_addergraph(nlevels)
    print "*"*80, "\nSCHED:", sched_type
    sched = sched_type(plasm)
    sched.execute(niter)
    print sched.stats()
    print "RESULT:", outnode.outputs.out
    shouldbe = float(2**nlevels * niter)
    print "expected:", shouldbe
    assert outnode.outputs.out == shouldbe

def test_plasm(nlevels, nthreads, niter):
    for sched in [ecto.Scheduler]:
        test_plasm_impl(sched, nlevels, nthreads, niter)

if __name__ == '__main__':
    #test_plasm(9, 4, 10000)
    test_plasm(10, 8, 10000)
    #test_plasm(9, 12, 10000)
    #test_plasm(6, 6, 6000)
    #test_plasm(8, 1, 5)
    #test_plasm(9, 64, 100)
    #test_plasm(10, 8, 10)
    #test_plasm(11, 8, 10)





