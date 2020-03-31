#!/usr/bin/env python
from ecto import Constant
from ecto.ecto_test import Multiply
from ecto.opts import CellYamlFactory
import yaml
import os

m_factory = CellYamlFactory(Multiply, 'mult')

#write to file
tmp_name = os.tmpnam()
with open(tmp_name,'w') as f:
    m_factory.dump(f)

#to string
print '#some yaml\n',m_factory.dump() 

#read from file
parsed = {}
with open(tmp_name,'r') as f:
    parsed = yaml.load(f)
os.remove(tmp_name)

#create an instance from a parsed yaml file
m = m_factory.load(parsed,cell_name='my_mult_instance')
print m, m.name(), m.params.factor
