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
import sys, ecto
import ecto.ecto_test as ecto_test
from util import fail
from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward

class MyBlackBox(ecto.BlackBox):
    ''' A simple black box that doesn't really do anything.
    '''

    @classmethod
    def declare_cells(cls, p):
        return {'gen': CellInfo(python_class=ecto_test.Generate),
                'inc': CellInfo(python_class=ecto_test.Increment)
               }

    @classmethod
    def declare_direct_params(cls, p, **kwargs):
        p.declare("fail", "Should i fail or should i go.", False)

    @classmethod
    def declare_forwards(cls, params):
        p = {'gen': 'all',
             'inc': [Forward(key='amount')]}
        i = {}
        o = {'inc': [Forward(key='out', new_key='value', new_doc='New docs')]}

        return (p,i,o)

    def configure(self, p, i, o):
        self.fail = p.at('fail').val
        self.printer = ecto_test.Printer()

    def connections(self, p):
        graph = [
                self.gen["out"] >> self.inc["in"],
                self.inc["out"] >> self.printer["in"]
               ]
        if self.fail:
            graph += [ ecto_test.ExceptInConstructor() ]
        return graph

def test_bb(options):
    mm = MyBlackBox(start=10, step=3, amount=2, niter=2)
    plasm = ecto.Plasm()
    plasm.insert(mm)
    options.niter = 5
    run_plasm(options, plasm)
    # final value is start + step*(2*5-1)+amount
    assert mm.outputs.value == 39
    run_plasm(options, plasm)
    # final value is start + step*(2*(5+5)-1)+amount
    assert mm.outputs.value == 69

def test_bb_fail(options):
    mm = MyBlackBox("MaMaMa", start=10, step=3, fail=True)
    print mm.__doc__
    assert 'fail' in  mm.__doc__
    assert mm.name() == 'MaMaMa'
    plasm = ecto.Plasm()
    plasm.insert(mm)
    try:
        run_plasm(options, plasm)
        fail()
    except ecto.CellException, e:
        print "Good:"
        print str(e)
        assert "I hate life" in str(e)

def test_command_line_args():
    import argparse
    from ecto.opts import cell_options
    parser = argparse.ArgumentParser()
    bb_factory = cell_options(parser, MyBlackBox, 'bb')
    parser.print_help()

def test_command_line_args2():
    import argparse
    from ecto.opts import cell_options, CellYamlFactory
    import yaml
    parser = argparse.ArgumentParser()
    bb_factory = cell_options(parser, MyBlackBox, 'bb')
    args = parser.parse_args(['--bb_start', '102'])
    mm = bb_factory(args)
    assert mm.params.start == 102

def test_yaml():
    from ecto.opts import CellYamlFactory
    import yaml
    bb_yaml = CellYamlFactory(MyBlackBox(start=54), 'bb')
    bb_yaml.dump(sys.stdout)
    mm = bb_yaml.load(yaml.load(bb_yaml.dump()), 'bb')
    print mm.params.start
    assert mm.params.start == 54

class MyBlackBox2(ecto.BlackBox):
    '''
    This BlackBox tests:
    - a BlackBox within a Blacbox
    - some parameters are implicitly declared
    - no declare_direct_params
    - two cells of the same type are part of it
    '''

    @classmethod
    def declare_cells(cls, p):
        return {'gen': CellInfo(python_class=MyBlackBox, params={'start':20}),
                'inc': CellInfo(python_class=ecto_test.Increment)
               }

    @classmethod
    def declare_forwards(cls, p):
        p = {'gen': [Forward(key='step'), Forward(key='amount',new_key='amount1')],
             'inc': [Forward(key='amount',new_key='amount2')]}
        i = {}
        o = {'inc': [Forward(key='out', new_key='value', new_doc='New docs')]}
        return (p,i,o)

    def configure(self, p, i, o):
        self.printer = ecto_test.Printer()

    def connections(self, p):
        graph = [
                self.gen["value"] >> self.inc["in"],
                self.inc["out"] >> self.printer["in"]
               ]
        return graph

def test_bb2(options):
    # start is going to be ignored as it is set to 20 by default
    mm = MyBlackBox2(start=0, step=3, amount1=10, amount2=50)
    # make sure the declare functions work
    p=ecto.Tendrils()
    i=ecto.Tendrils()
    o=ecto.Tendrils()
    mm.declare_params(p)
    mm.declare_io(p,i,o)
    # run the BlackBox
    plasm = ecto.Plasm()
    plasm.insert(mm)
    options.niter = 5
    run_plasm(options, plasm)
    # final value is start + step*(5-1)+amount1+amount2
    assert mm.outputs.value == 92
    run_plasm(options, plasm)
    # final value is start + step*(5+5-1)+amount1+amount2
    assert mm.outputs.value == 107

class MyBlackBox3(ecto.BlackBox):
    pass
class MyBlackBox4(ecto.BlackBox):
    def connections(self, p):
        return []
class MyBlackBox5(ecto.BlackBox):
    def declare_cells(self, p):
        return {'gen': CellInfo(python_class=ecto_test.Generate),
                'inc': CellInfo(python_class=ecto_test.Increment)}
    @classmethod
    def declare_direct_params(cls, p, **kwargs):
        p.declare("fail", "Should i fail or should i go.", False)
    @classmethod
    def declare_forwards(cls, params):
        return ({},{},{})
    def connections(self, p):
        return [self.gen["out"] >> self.inc["in"]]
class MyBlackBox6(ecto.BlackBox):
    @classmethod
    def declare_cells(cls, p):
        return {'gen': CellInfo(python_class=ecto_test.Generate),
                'inc': CellInfo(python_class=ecto_test.Increment)}
    def declare_direct_params(self, p, **kwargs):
        p.declare("fail", "Should i fail or should i go.", False)
    @classmethod
    def declare_forwards(cls, params):
        return ({},{},{})
    def connections(self, p):
        return [self.gen["out"] >> self.inc["in"]]
class MyBlackBox7(ecto.BlackBox):
    @classmethod
    def declare_cells(cls, p):
        return {'gen': CellInfo(python_class=ecto_test.Generate),
                'inc': CellInfo(python_class=ecto_test.Increment)}
    @classmethod
    def declare_direct_params(cls, p, **kwargs):
        p.declare("fail", "Should i fail or should i go.", False)
    def declare_forwards(self, params):
        return ({},{},{})
    def connections(self, p):
        return [self.gen["out"] >> self.inc["in"]]

def test_bb_static():
    for BB in [MyBlackBox3, MyBlackBox4, MyBlackBox5, MyBlackBox6, MyBlackBox7]:
        try:
            mm = BB("MaMaMa")
            fail()
        except ecto.BlackBoxError as e:
            print "Good:"
            print str(e)

if __name__ == '__main__':
    test_command_line_args()
    test_command_line_args2()
    test_yaml()
    from ecto.opts import scheduler_options, run_plasm
    import argparse
    parser = argparse.ArgumentParser()
    scheduler_options(parser)
    options = parser.parse_args()
    test_bb(options)
    test_bb_fail(options)
    test_bb_static()

    test_bb2(options)
