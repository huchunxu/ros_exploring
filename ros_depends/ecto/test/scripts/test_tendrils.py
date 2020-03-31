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

def test_tendrils():
    t = ecto.Tendrils()
    t.declare("Hello","doc str",6)
    assert t.Hello == 6
    assert t["Hello"] == 6
    t.declare("x","a number", "str")
    assert len(t) == 2
    assert t["x"] == "str"
    assert t.x == "str"
    #test the redeclare
    try:
        t.declare("Hello","new doc", "you")
        util.fail()
    except ecto.TendrilRedeclaration, e:
        print str(e)
        assert('TendrilRedeclaration' in str(e))
    try:
        #read error
        t.nonexistant = 1
        util.fail()
    except ecto.NonExistant, e:
        print str(e)
        assert re.findall("tendril_key.*nonexistant", str(e))
    try:
        #index error
        print t["nonexistant"]
        util.fail()
    except ecto.NonExistant, e:
        print str(e)
        assert re.findall("tendril_key.*nonexistant", str(e))

    assert len(t.keys()) == 2
    assert len(t.values()) == 2

    print t
    #by value
    _x = t.x
    _x = 10
    assert t.x != 10
    x = t.x
    t.x = 11
    assert x != 11
    #by reference
    x = t.at("x")
    t.x = 13
    assert x.val == 13

    t.x = 17
    assert t.x == 17
    t.x = 199
    t.x = 15
    print t.x
    assert t.x == 15

if __name__ == '__main__':
    test_tendrils()

