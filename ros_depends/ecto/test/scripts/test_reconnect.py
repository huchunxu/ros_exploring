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
'''
Created on Apr 8, 2011

@author: erublee
'''
import ecto
import ecto.ecto_test as ecto_test

def test_one_to_many():
    plasm = ecto.Plasm()
    g = ecto_test.Generate(start=2, step=2)
    modules = []
    for x in range(0,5):
        m = ecto_test.Multiply(factor=2)
        plasm.connect(g,"out",m,"in")
        modules.append(m)
        
    sched = ecto.Scheduler(plasm)
    sched.execute(niter=1)
    for x in modules:
        #print x.outputs.out
        assert(x.outputs.out == 4)
    sched.execute(niter=1)
    for x in modules:
        #print x.outputs.out
        assert(x.outputs.out == 8)
    

def test_reconnect():
    plasm = ecto.Plasm()
    g = ecto_test.Generate(start=2, step=2)    
    m = ecto_test.Multiply(factor=2)
    m2 = ecto_test.Multiply(factor=2)
    gather = ecto_test.Gather_double(n=2)
    plasm.connect(g, "out", m , "in")
    plasm.connect(g, "out", m2 , "in")
    try:
        plasm.connect(m2,"out",m,"in")
        util.fail("reconnection bad...")
    except RuntimeError,e:
        pass
        #print "Reconnect caught: ",e
    plasm.connect(m2, "out", gather , "in_0000")
    plasm.connect(m, "out", gather , "in_0001")
    try:
        plasm.connect(m2, "out", gather , "in_0001")
        util.fail()
    except RuntimeError,e:
        pass
    plasm.disconnect(m, "out", gather , "in_0001")
    plasm.connect(m, "out", gather , "in_0001")

    #ecto.view_plasm(plasm)
    #check some values
    sched = ecto.Scheduler(plasm)
    sched.execute(niter=1)
    print gather.outputs.out
    assert(gather.outputs.out == 2 *(2*2))
    
if __name__ == "__main__":
    test_reconnect()
    test_one_to_many()

