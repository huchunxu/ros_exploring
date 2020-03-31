#!/usr/bin/env python
import ecto, sys
import ecto.ecto_test as ecto_test

plasm = ecto.Plasm()
 
gen = ecto_test.Generate("Gen", step=1.0, start=0.0)
inc = ecto_test.Increment("Increment 0", delay=100)
printer = ecto_test.Printer("Printy")
 
plasm.connect(gen[:] >> inc[:],
              inc[:] >> printer[:])
  
sched = ecto.schedulers.Threadpool(plasm)
sched.execute_async(niter=10)
    
from IPython.Shell import IPShellEmbed
ipshell = IPShellEmbed()
ipshell()
