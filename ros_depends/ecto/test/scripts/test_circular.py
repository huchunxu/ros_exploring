#!/usr/bin/env python
import ecto
from ecto import ecto_test

def test_circular():
    a1 = ecto_test.Add()
    a2 = ecto_test.Add()
    p = ecto.Plasm()
    p.connect(a1['out'] >> a2['left'],
              a2['out'] >> a1['right']
              )

    sched = ecto.Scheduler(p)
    try:
        sched.execute(niter=1)
        fail("that should have thrown")
    except ecto.EctoException as e: # Thrown when Lumberg's "BFS" is used.
        print "okay, threw"
    except ValueError as e: # Thrown when boost::topological_sort() is used.
        print "okay, threw"

if __name__ == '__main__':
    test_circular()
