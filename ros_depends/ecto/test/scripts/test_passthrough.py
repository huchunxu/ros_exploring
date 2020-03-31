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

def test_constant2():
    print "test running.."
    plasm = ecto.Plasm()
    c = ecto.Constant(value=0.50505)
    m = ecto_test.Multiply(factor=3.3335)
    passthrough = ecto.Passthrough()
    print passthrough.__doc__
    print ">>>>DOC>>>>", c.__doc__
    pr = ecto_test.Printer()

    p = ecto.Plasm()

    plasm.connect(c[:] >> m[:],
                  m[:] >> passthrough[:],
                  passthrough[:] >> pr[:]
                  )

    plasm.execute()

    assert m.outputs.out == (0.50505 * 3.3335)
    plasm.execute()

    assert m.outputs.out == (0.50505 * 3.3335)
    
def test_constant():
    print "test running.."
    plasm = ecto.Plasm()
    c = ecto.Constant(value=0.50505)
    m = ecto_test.Multiply(factor=3.3335)
    passthrough = ecto_test.PassthroughAny()
    print passthrough.__doc__
    print ">>>>DOC>>>>", c.__doc__
    pr = ecto_test.Printer()

    p = ecto.Plasm()

    plasm.connect(c[:] >> m[:],
                  m[:] >> passthrough[:],
                  passthrough[:] >> pr[:]
                  )

    plasm.execute()

    assert m.outputs.out == (0.50505 * 3.3335)
    plasm.execute()

    assert m.outputs.out == (0.50505 * 3.3335)

if __name__ == '__main__':
    test_constant()
    test_constant2()



