#!/usr/bin/env python
import ecto
from ecto.ecto_test import Generate, Add

g = Generate("Generator", step=1.0, start=1.0)
add = Add()
source, sink = ecto.EntangledPair(value=add.inputs.at('left'))

plasm = ecto.Plasm()
plasm.connect(source[:] >> add['left'],
              g[:] >> add['right'],
              add[:] >> sink[:]
              )

for i in range(0, 5):
    plasm.execute(niter=1)
    print add.outputs.out
