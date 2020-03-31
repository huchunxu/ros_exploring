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

import os
import sys
from multiprocessing import Process, Pipe, current_process, freeze_support
import StringIO

#http://docs.python.org/library/multiprocessing.html

class Sender(ecto.Cell):
    """ A python module that does not much."""
    @staticmethod
    def declare_params(params):
        params.declare("conn", "A pipe connection.", None)
        params.declare("file", "A file-like object.", StringIO.StringIO())

    @staticmethod
    def declare_io(params, inputs, outputs):
        inputs.declare("file", "A file like object", None)
        pass

    def configure(self, params):
        pass

    def process(self, inputs, outputs):
        f = StringIO.StringIO()
        f.write('hello there')
        f.seek(0)
        self.params.conn.send(f)
        return 0

def f(conn):
    s = Sender(conn=conn)
    plasm = ecto.Plasm()
    plasm.insert(s)
    sched = ecto.Scheduler(plasm)
    sched.execute(niter=10)

def g(conn):
    while True:
        file = conn.recv()
        for x in file:
            print x
            assert x == 'hello there'

if __name__ == '__main__':
    freeze_support()
    f_conn, g_conn = Pipe()
    p1 = Process(target=f, args=(f_conn,))
    p2 = Process(target=g, args=(g_conn,))

    p1.start()
    p2.start()
    p1.join()
    p2.terminate()
