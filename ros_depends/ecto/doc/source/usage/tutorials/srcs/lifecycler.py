#!/usr/bin/env python
import ecto

def doit():
    print "**** pre import of ecto_lifecycle ********"
    import ecto.ecto_lifecycle as ecto_lifecycle
    print "**** post import of ecto_lifecycle *******"

    print "**** pre allocation of LifeCycle cell ********"
    lifecycle = ecto_lifecycle.LifeCycle() #allocate a cell
    print "**** post allocation of LifeCycle cell *******"

    plasm = ecto.Plasm()
    print "**** pre insert ********"
    plasm.insert(lifecycle)
    print "**** post insert *******"

    print "**** pre execute ********"
    sched = ecto.Scheduler(plasm)
    sched.execute(1)
    print "**** post execute *******"

print "**** pre scope ********"
doit()
print "**** post scope *******"
