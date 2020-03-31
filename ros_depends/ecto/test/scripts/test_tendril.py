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

def test_tendril():
    print "here"
    tendril = ecto.Tendril()
    tendril.set(5)
    t = ecto.Tendril()
    t.val = 5
    #t.notify()
    #tendril.notify()
    assert t.val == t.get()
    assert t.val == 5
    assert tendril.val == t.val
    assert tendril.get() == 5
    assert tendril.get() == t.get()
    tendril.val = 10
    t.val = tendril.val
    t.val = "hi"
    t.notify()
    assert tendril.val != "hi"
    assert t.val == t.get()
    assert t.val == "hi"
    
def test_tendril_defs():
    t1 = ecto.Tendril()
    t2 = ecto.Tendril()
    t1.val = 10
    t1.notify()
    t1 = t2
    assert t2.val == t1.val
    t2.val = 13
    t2.notify()
    assert t1.val == 13
    print t1.doc
    print t1.type_name
    print t1.val
    print t1.get()
    t1.set("foo")
    t1.notify()
    print t1.val
    assert t2.val == t1.val

def test_cpp_python_tendril():
    x = ecto.Tendril.createT('int')
    x.val = 10
    x.notify()
    t1 = ecto.Tendril()
    #this connection should force the t1 to become a native type.
    t1 = x
    t1.val = 20
    t1.notify()
    assert t1.type_name == x.type_name
    assert x.val == 20

def test_python_serialization():
    y = ecto.Tendril(None) #empty tendril
    x = ecto.Tendril.createT('std::string')
    x.val = 'UuUuU'
    y.load(x.save())
    assert y.type_name == 'std::string'
    assert y.val == x.val
    print y.val
    sy = y.save()
    print len(sy)
    print len(y.val)
    import binhex
    print binhex.binascii.hexlify(sy)

if __name__ == '__main__':
    test_tendril()
    test_tendril_defs()
    test_cpp_python_tendril()
    test_python_serialization()


