#!/usr/bin/env python
import ecto
from ecto import If, TrueEveryN
from ecto.ecto_test import Generate

plasm = ecto.Plasm()
g = Generate("Generator", step=1.0, start=1.0)
if_g = If(cell=g)
truer = TrueEveryN(n=3,count=3)
plasm.connect(truer['flag'] >> if_g['__test__']
              )
plasm.execute(niter=9)
assert g.outputs.out == 3 #should have only called execute 3 times.
