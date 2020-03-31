#!/usr/bin/env python
import ecto #ecto core library
import ecto.hello_ecto as hello_ecto #a user library, that has a few ecto modules

plasm = ecto.Plasm()
r = hello_ecto.Reader()
p1 = hello_ecto.Printer(str="default")
p2 = hello_ecto.Printer(str="default")
plasm.connect(
              r["output"] >> (p1["str"],p2["str"])
              )

if __name__ == '__main__':
    plasm.save('printy.plasm')

