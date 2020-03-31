#!/usr/bin/python
import ecto
import ecto.ecto_test as ecto_test

plasm = ecto.Plasm()

gen = ecto_test.Generate()
inc = ecto_test.Increment()
mul = ecto_test.Multiply()
add = ecto_test.Add()

plasm.connect(gen[:] >> inc[:],
              gen[:] >> mul[:],
              mul[:] >> add['left'],
              inc[:] >> add['right'],
              )


