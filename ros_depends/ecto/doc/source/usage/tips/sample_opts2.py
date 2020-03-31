#!/usr/bin/env python
from ecto import Constant
from ecto.ecto_test import Multiply
import argparse

from ecto.opts import cell_options

parser = argparse.ArgumentParser(description='My awesome program thing.')

#add our cells to the parser
multiply_factory = cell_options(parser, Multiply, prefix='mult')
const_factory = cell_options(parser, Constant(value=0.50505), prefix='const')

args = parser.parse_args()

#use the factories in conjunction with the parsed arguments, to create our cells.
c = const_factory(args)
m = multiply_factory(args)

# ... construct graph do whatever else...
