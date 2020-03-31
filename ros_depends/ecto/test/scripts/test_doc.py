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
import pyecto

def test_doc():
    scatter = ecto_test.Scatter(n=6, x=3)
    gather = ecto_test.Gather(n=3)
    gather2 = ecto_test.Gather(n=3)
    plasm = ecto.Plasm()
    plasm.connect(
                  scatter["out_0000","out_0001","out_0002"] >> gather[:],
                  scatter["out_0003","out_0004","out_0005"] >> gather2[:]
                  )
    plasm.execute()
    result = gather.outputs.out
    assert(result == 9) # 3 * 3
    assert scatter.__doc__ != None
    assert gather.__doc__ != None
    assert type(plasm.viz()) == str

def test_inspection():
    assert len(ecto.list_cells(ecto_test)) >2
    assert len(ecto.list_cells(pyecto)) > 0

def test_short_doc():
    add = ecto_test.Add
    assert add.short_doc == 'Add two doubles together.'
    assert add.short_doc in add.__doc__
    assert 'Left hand operand' in add.__doc__
    assert 'Parameters:' not in add.__doc__
    assert 'Outputs:' in add.__doc__
    assert 'Inputs:' in add.__doc__
    assert 'The result.' in add.__doc__

if __name__ == '__main__':
    test_doc()
    test_inspection()
    test_short_doc()

