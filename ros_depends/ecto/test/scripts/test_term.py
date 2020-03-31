#!/usr/bin/env python

import ecto, signal, ecto_test, sys

terminate_req=False
def terminate(sign, frame):
    global terminate_req
    print 'termination requested'
    terminate_req = True
    #ecto_test.abort() #for gdb break point

#install callback
signal.signal(signal.SIGTERM, terminate)
signal.signal(signal.SIGQUIT, terminate)

scatter = ecto_test.Scatter(n=3, x=3)
gather = ecto_test.Gather(n=3)

plasm = ecto.Plasm()
plasm.connect(scatter[:] >> gather[:])
sched = ecto.Scheduler(plasm)

count = 0
while (terminate_req == False) and (sched.execute(niter=1)):
    count += 1

print count
