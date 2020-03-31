#!/usr/bin/env python
# this is the same as plasms.py except it is meant to be run from `make doc`
import ecto
from ecto.tutorial import Increment, Add

i1 = Increment('Inc 1', start=18)
i2 = Increment('Inc 2', start=20)
add = Add('Add')

plasm = ecto.Plasm()
plasm.connect(i1['output'] >> add['a'],
              i2['output'] >> add['b'],
              )

sched = ecto.Scheduler(plasm)
sched.execute(niter=2)

print add.outputs.output
