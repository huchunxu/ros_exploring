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
from pyecto import MyModule
import time


def test_python_module():
    mod = MyModule("mymodule", text="spam")
    mod.configure(mod.params, mod.inputs, mod.outputs)
    assert mod.text == "spam"
    assert mod.params.text == "spam"
    mod.process(mod.inputs,mod.outputs)
    print mod.outputs.out
    assert mod.outputs.out == "spam"*2

def test_python_module_plasm(Schedtype):
    print "*"*80
    print Schedtype
    mod = MyModule(text="spam")
    mod.configure(mod.params, mod.inputs, mod.outputs)
    g = ecto_test.Generate(start = 1 , step =1)
    plasm = ecto.Plasm()
    plasm.connect(g,"out",mod,"input")
    sched = Schedtype(plasm)
    for i in range(1,5):
        print "HERE"
        sched.execute(niter=1)
        sched.prepare_jobs(niter=1)
        sched.run()
        assert g.outputs.out == i*2
        assert mod.outputs.out == "spam"*i*2

    sched.execute(niter=1)
    sched.prepare_jobs(niter=1)
    sched.run()
    assert g.outputs.out == 10
    assert mod.outputs.out == "spam"*10

if __name__ == '__main__':
    test_python_module()
    map(test_python_module_plasm, [ecto.Scheduler])

