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

import ecto, sys, util
from ecto.ecto_test import *

p = ecto.Plasm()

emitters = [Emit_bool, Emit_int, Emit_float, Emit_Struct, Emit_string]
acceptors = [Accept_bool, Accept_int, Accept_float, Accept_Struct, Accept_string]

def do(E, A):
    p = ecto.Plasm()
    e = E()
    a = A()
    ltype = E.__name__.split('_')[1]
    rtype = A.__name__.split('_')[1]
    if ltype == rtype:
        p.connect(e[:] >> a[:])
        return

    def throw():
        p.connect(e[:] >> a[:])
        raise "should have thrown"

    print "connecting, should throw..."
    try:
        throw()
        util.fail()
    except ecto.ValueNone, ex:
        util.fail()
    except ecto.ValueRequired, ex:
        util.fail()
    except ecto.EctoException, ex:
        print "CAUGHT! ok!", ex
        print sys.exc_info()

    try:
        throw()
        util.fail()
    except ecto.TypeMismatch, ex:
        print "CAUGHT! ok!", ex
        print sys.exc_info()

    try:
        throw()
        util.fail()
    except RuntimeError, ex:
        print "okay!", ex
        print sys.exc_info()

def valnone(A):
    p = ecto.Plasm()
    e = Emit_none()
    a = A()
    # doesn't throw yet...
    p.connect(e[:] >> a[:])
    try:
        p.execute(niter=1)
    except ecto.ValueNone, va:
        print "yeah, got ValueNone error"

def typeconv(E):
    p = ecto.Plasm()
    e = E()
    a = Accept_none()
    p.connect(e[:] >> a[:])
    try:
        p.execute(niter=1)
        util.fail()
    except ecto.TypeMismatch, va:
        print "yeah, got typeconv error"

for E in emitters:
    typeconv(E)

sys.exit(0)
for A in acceptors:
    valuenone(A)


for E in emitters:
    for A in acceptors:
        print E, A
        do(E, A)



