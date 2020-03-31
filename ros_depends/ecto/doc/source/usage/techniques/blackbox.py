#!/usr/bin/env python
import ecto
from ecto.tutorial import Increment, Add
from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward

class MyBlackBox(ecto.BlackBox):
    """
    We encapsulate the plasm from the hello_tutorial by exposing the
    start value of 'i2' as a parameter to the BlackBox and forwarding
    the output of the 'add' cell
    """
    @staticmethod
    def declare_cells(_p):
        """
        Implement the virtual function from the base class
        Only cells from which something is forwarded have to be declared
        """
        cells = {}
        # 'i2' has its start value exposed to the user so only a type is given
        cells['i2'] = CellInfo(Increment, name='Inc 2')
        # 'add' is always the same so we could define with a CellInfo(Add, name='Add') or
        # just with an instance
        cells['add'] = Add('Add')

        return cells

    @staticmethod
    def declare_forwards(_p):
        """
        Implement the virtual function from the base class
        """
        # we forward the start parameter of the cells 'i2' but the user will
        # see it as 'start2'
        p = {'i2': [Forward('start',new_key='start2',new_default=20)]}

        # there are no inputs to expose to the user
        i={}

        # we forward all the outputs from add to the user
        o = {'add': 'all'}

        return (p, i, o)

    def configure(self, _p,_i,_o):
        # implicitly, 'add' and 'i2' will be created as they have been declared
        # only 'i1' needs to be defined
        self.i1 = Increment('Inc 1', start=18)

    def connections(self, _p):
        # define the connections like you would for the plasm
        return [ self.i1['output'] >> self.add['a'],
                 self.i2['output'] >> self.add['b'] ]

# create an instance of my BlackBox
my_blackbox = MyBlackBox(start2=18)
# create a plasm that only contains the BlackBox
plasm = ecto.Plasm()
plasm.insert(my_blackbox)
# execute the plasm
plasm.execute(niter=2)

# display the output name 'output' in the outputs of cell 'add'
print my_blackbox.outputs.output
