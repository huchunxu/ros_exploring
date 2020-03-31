#!/usr/bin/env python

import ecto
import ecto.ecto_test as ecto_test

s1 = ecto.Strand()  # No cells that have this strand will run concurrenly

plasm = ecto.Plasm()
gen = ecto_test.Generate("Gen", step=1.0, start=1.0)
noncurr = ecto_test.DontCallMeFromTwoThreads("Unsafe0", strand=s1)
plasm.connect(gen['out'] >> noncurr['in'])

for k in range(3):
    next = ecto_test.DontCallMeFromTwoThreads("Unsafe%u" % (k+1), strand=s1)
    plasm.connect(noncurr['out'] >> next['in'])
    noncurr = next

printer = ecto_test.Printer()
plasm.connect(noncurr['out'] >> printer['in'])

if __name__ == '__main__':
    sched = ecto.Scheduler(plasm)
    sched.execute(niter=2)

