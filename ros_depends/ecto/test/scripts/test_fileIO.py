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
import StringIO

cards = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
filetext = '\n'.join([str(x) for x in cards]) + '\n'
def test_fileO(Scheduler, file_like_object, realfile=False):
    global cards, filetext
    plasm = ecto.Plasm()
    printer = ecto_test.FileO(file=ecto.ostream(file_like_object))
    dealer = ecto.Dealer(tendril=printer.inputs.at('input'), iterable=cards)
    plasm.connect(dealer['out'] >> printer['input'])
    sched = Scheduler(plasm)
    sched.execute()

    if not realfile:
        file_like_object.seek(0)
        result = ''.join([x for x in file_like_object])
        print result
        assert result == filetext

def test_fileI(Scheduler, file_like_object, realfile=False):
    global cards
    plasm = ecto.Plasm()
    if not realfile:
        file_like_object.writelines(filetext)
        file_like_object.seek(0)
    reader = ecto_test.FileI(file=ecto.istream(file_like_object))
    printer = ecto_test.Printer()
    plasm.connect(reader[:] >> printer[:])
    sched = Scheduler(plasm)
    sched.execute()
    assert reader.outputs.output == cards[-1]

def test_io_fake(Scheduler):
    outty = StringIO.StringIO()
    test_fileO(Scheduler, outty)
    inny = StringIO.StringIO()
    test_fileI(Scheduler, inny)

def test_io_real(Scheduler):
    with open('cards.txt', 'w') as f:
        test_fileO(Scheduler, f, realfile=True)
    with open('cards.txt', 'r') as f:
        test_fileI(Scheduler, f, realfile=True)
    import os
    os.remove('cards.txt')

def test_io_stdo(Scheduler=ecto.Scheduler):
    import sys
    test_fileO(Scheduler, sys.stdout, realfile=True)

if __name__ == '__main__':
    for x in [ecto.Scheduler]:
        print " >>>>>>>>> Start sched >>>>>>>>>>", str(x)
        test_io_fake(x)
        test_io_real(x)
        test_io_stdo(x)
        print "<<<<<<<<<< End sched <<<<<<<<<<<", str(x)


