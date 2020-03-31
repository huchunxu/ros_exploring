#!/usr/bin/env python
# we import out ecto module: it was built in the ecto_tutorial folder and it is named tutorial
import ecto_tutorial.tutorial as tutorial

# we get the cell that we named Hello in that module
printer = tutorial.Hello()
# execute the cell itself
printer.process()
