#!/usr/bin/env python
# 
# Copyright (c) 2014, Yujin Robot
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Yujin Robot nor the names of its
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

def test_nested_if():
    plasm = ecto.Plasm()
    g = ecto_test.Generate("Generator", step=1.0, start=1.0)
    inside_if = ecto.If(input_tendril_name="on_threes", cell=g)
    outside_if = ecto.If(input_tendril_name="on_twos", cell=inside_if)
    truer_on_threes = ecto.TrueEveryN(n=3,count=0)
    truer_on_twos = ecto.TrueEveryN(n=2,count=0)
    plasm.connect([
        truer_on_threes['flag'] >> outside_if['on_threes'],
        truer_on_twos['flag'] >> outside_if['on_twos']
    ])
    #for x in range(0,18):
    #    plasm.execute(niter=1)
    #    print("No of times executed: %s of %s" % (g.outputs.out, x))
        
    # executes on the very first iteration (count = 0) and once every 3*2 iterations thereafter
    plasm.execute(niter=18)
    assert g.outputs.out == 3 # should have only called execute 3 times.
    plasm.execute(niter=1)
    assert g.outputs.out == 4 # should have executed once more
if __name__ == '__main__':
    test_nested_if()



