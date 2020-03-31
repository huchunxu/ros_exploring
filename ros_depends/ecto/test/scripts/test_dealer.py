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

def test_dealer(Scheduler):
    print "*" *80
    print __name__, 'test_dealer', Scheduler
    plasm = ecto.Plasm()
    printer = ecto_test.Printer()
    cards = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    dealer = ecto.Dealer(tendril=printer.inputs.at('in'), iterable=cards)
    plasm.connect(dealer['out'] >> printer['in'])
    sched = Scheduler(plasm)
    print 'executing ...'
    sched.execute()
    print 'finished executing'
    assert dealer.outputs.at('out').type_name == 'double'
    assert dealer.outputs.out == 10

def test_dealer_heterogenous_type_fail(Scheduler):
    print "*" * 80
    print __name__, 'test_dealer_heterogenous_type_fail', Scheduler
    printer = ecto_test.Printer()
    cards = [1, 2, 3, 4, 5, 'hello', 7, 8, 9, 10]
    dealer = ecto.Dealer(tendril=printer.inputs.at('in'), iterable=cards)
    plasm = ecto.Plasm()
    plasm.connect(dealer['out'] >> printer['in'])
    sched = Scheduler(plasm)
    try:
        sched.execute()
        assert False == " Should have thrown."
    except ecto.FailedFromPythonConversion, e:
        print "Threw as expected:", str(e)
        assert re.findall('cpp_typename.*double', str(e))

if __name__ == '__main__':
    test_dealer(ecto.Scheduler)
    test_dealer_heterogenous_type_fail(ecto.Scheduler)

