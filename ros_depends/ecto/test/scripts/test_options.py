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
import yaml
import argparse
from ecto.opts import scheduler_options, CellYamlFactory, cell_options, run_plasm

parser = argparse.ArgumentParser(description='My awesome program thing.')
parser.add_argument('-i,--input', metavar='IMAGE_FILE', dest='imagefile',
                    type=str, default='', help='an image file to load.')
group = parser.add_argument_group('ecto scheduler options')
scheduler_options(group, default_niter=2)


multiply_factory = cell_options(parser, ecto_test.Multiply, prefix='mult')
const_factory = cell_options(parser, ecto.Constant(value=0.50505), prefix='const')

options = parser.parse_args()
print options.mult_factor
assert options.mult_factor == 3.14
c = const_factory(options)
m = multiply_factory(options)

cyaml = CellYamlFactory(c,'const')
print cyaml.dump()
c = cyaml.load(yaml.load(cyaml.dump()))
assert c.params.value == 0.50505

pr = ecto_test.Printer()
plasm = ecto.Plasm()
plasm.connect(c[:] >> m[:],
              m[:] >> pr[:]
              )

run_plasm(options, plasm, locals=vars())

print m.outputs.out
assert m.outputs.out == 1.585857
