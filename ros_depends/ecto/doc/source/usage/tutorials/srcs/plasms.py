#!/usr/bin/env python
import ecto
from ecto_tutorial.tutorial import Increment, Add

# create three Cell objects
i1 = Increment('Inc 1', start=18)
i2 = Increment('Inc 2', start=20)
add = Add('Add')

# create an empty plasm: a graph
plasm = ecto.Plasm()
# create connections inside that graph: the output named 'output'
# of cell 'i1' is connected to the inut named 'a' of cell 'add'
# and reciprocally with cell 'i2'.
plasm.connect(i1['output'] >> add['a'],
              i2['output'] >> add['b'],
              )

# create a scheduler that will run that plasm
sched = ecto.Scheduler(plasm)
# execute the plasm
sched.execute(niter=2)

# display the output name 'output' in the outputs of cell 'add'
print add.outputs.output
