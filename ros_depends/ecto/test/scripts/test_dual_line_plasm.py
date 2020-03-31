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

def test_dual_line_plasm(nlevels):
    plasm = ecto.Plasm()

    gen = ecto_test.Generate(step=1.0, start=0.0)
    incl, incr = ecto_test.Increment(), ecto_test.Increment()

    plasm.connect(gen, "out", incl, "in")
    plasm.connect(gen, "out", incr, "in")

    for j in range(nlevels-1): # one set of incs has already been added
        print j
        inc_nextl, inc_nextr = ecto_test.Increment(), ecto_test.Increment()
        plasm.connect(incl, "out", inc_nextl, "in")
        plasm.connect(incr, "out", inc_nextr, "in")
        incl, incr = inc_nextl, inc_nextr

    add = ecto_test.Add()
    plasm.connect(incl, "out", add, "left")
    plasm.connect(incr, "out", add, "right")
    printer = ecto_test.Printer()
    plasm.connect(add, "out", printer, "in")
    
    sched = ecto.Scheduler(plasm)
    sched.execute(niter=1)
    result = add.outputs.out
    print "result=", result
    assert(result == nlevels * 2)

    sched.execute(niter=2)
    result = add.outputs.out
    print "iter2 result=", result
    assert result == (nlevels + 2) * 2

    sched.execute(niter=3)
    result = add.outputs.out
    print "iter3 result=", result
    assert result == (nlevels + 5) * 2

if __name__ == '__main__':
    test_dual_line_plasm(10)
    test_dual_line_plasm(100)




