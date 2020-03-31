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
import ecto.ecto_test as ecto_test

def test_required_param():
    plasm = ecto.Plasm()
    print "<DOC>", ecto_test.RequiredParam.__doc__, "</DOC>"
    #test
    assert "REQUIRED" in ecto_test.RequiredParam.__doc__
    #test doc default value printing printing
    assert "2.1253" in ecto_test.RequiredParam.__doc__
    try:
        req = ecto_test.RequiredParam("Required")
        print "egh, didn't throw"
        util.fail()
    except RuntimeError, e:
        print "Yup, there is our throw:", e
        
    req = ecto_test.RequiredParam("Required", x=2.2)
    assert req.params.at("x").required == True

    gen = ecto_test.Generate("Generator")
    printer = ecto_test.Printer("Printy")
    plasm.connect(gen[:] >> req[:],
                   req[:] >> printer[:])
    plasm.execute(niter=3)
    assert req.outputs.out == 6.2

if __name__ == '__main__':
    test_required_param()





