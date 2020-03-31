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
from __future__ import print_function
import ecto
import ecto.ecto_test as ecto_test
import subprocess
import tempfile
import unittest

class TestLoadSavePlasm(unittest.TestCase):
    FILE = tempfile.mkstemp(text=True)[1]
    #def __del__(self):
        #os.remove(FILE)

    def test_0_save_plasm(self):
        n_nodes = 5
        incdelay= 1
        plasm = ecto.Plasm()

        gen = ecto_test.Generate("Gen", step=1.0, start=0.0)
        inc = ecto_test.Increment("Increment 0", delay=incdelay)

        plasm.connect(gen, "out", inc, "in")

        for j in range(n_nodes-1): # one has already been added
            inc_next = ecto_test.Increment("Increment_%u" % (j+1), delay=incdelay)
            plasm.connect(inc, "out", inc_next, "in")
            inc = inc_next

        printer = ecto_test.Printer("Printy")
        plasm.connect(inc, "out", printer, "in")
        plasm.save(self.FILE)

    def test_1_load_plasm(self):
        plasm = ecto.Plasm()

        plasm.load(self.FILE)

        print(plasm.viz())

#    def test_2_run_plasm(self):
#        print(self.FILE)
#        print('rosrun ecto plasm_loader ' + self.FILE + ' 8')
#        p = subprocess.Popen('rosrun ecto plasm_loader ' + self.FILE + ' 8', stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=T
#        out, err = p.communicate()
#        self.assertEqual(err, '')
#        print(out)
#        print(err)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('ecto_object_recognition_ros', 'test_actionlib', TestLoadSavePlasm)
