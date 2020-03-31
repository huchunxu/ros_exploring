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
import re

class shouldathrown: pass

def test_plasm():
    scatter = ecto_test.Scatter(n=3, x=3)
    scatter2 = ecto_test.Scatter(n=5, x=10)

    gather = ecto_test.Gather(n=3)
    gather2 = ecto_test.Gather(n=5)
    plasm = ecto.Plasm()

    p = ecto.Plasm()
    #test the old syntax.
    p.connect(scatter, "out_0000", gather, "in_0000")

    try:
        p2 = ecto.Plasm()
        p2.connect(scatter, "out_0000", gather, "idn_0001")
        util.fail()
    except ecto.NonExistant, e:
        print ">>>",e
        assert "'in_0000':type(int) 'in_0001':type(int) 'in_0002':type(int)" in str(e)
        print "(threw as expected)\n\n"

    try:
        p2 = ecto.Plasm()
        p2.connect(gather["out"] >> ecto_test.Printer(print_type="double")["in"])
        util.fail("Should not work as there is a type mismatch...")
    except ecto.TypeMismatch, e:
        print "type:",type(e)
        print ">>>",e
        assert re.findall("from_typename.*int", str(e))
        assert re.findall("to_typename.*double", str(e))
        print "(threw as expected)"

    try:
        p2 = ecto.Plasm()
        p2.connect(gather["out"],ecto_test.Printer(print_type="double")["in"])
        util.fail("Should not work.")
    except RuntimeError, e:
        print e
        assert 'Did you mean' in str(e)

    plasm.connect(scatter[:] >> gather[:],
                  scatter2[:] >> gather2[:],
                  gather["out"] >> ecto_test.Printer(print_type="int")["in"]
                  )

    plasm.execute()

    #tests introspection
    viz = plasm.viz()
    assert(type(viz) == str)

    result1 = gather.outputs.out
    print result1
    assert(result1 == 9) # 3 * 3
    result2 = gather2.outputs.out
    print result2
    assert(result2 == 50) # 5 * 10

    l = plasm.connections()
    print l

    plasm2 = ecto.Plasm()
    plasm2.connect(l)
    assert plasm.viz() == plasm2.viz()
    assert l == plasm2.connections()
    plasm2.execute()
    assert gather2.outputs.out == 50

def bad_syntax_errors():
    scatter = ecto_test.Scatter(n=3, x=3)
    scatter2 = ecto_test.Scatter(n=5, x=10)

    gather = ecto_test.Gather(n=3)
    gather2 = ecto_test.Gather(n=5)

    try:
        plasm = ecto.Plasm()
        plasm.connect(
                  scatter[:] >> gather2[:]
                  )
        util.fail("Should not work as there is a size mismatch...")
    except RuntimeError, e:
        print e

if __name__ == '__main__':
    test_plasm()
    bad_syntax_errors()




