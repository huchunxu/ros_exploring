#!/usr/bin/env python
import ecto #this must be imported before other ecto based python modules
import tutorial #this is our ecto module

#allocate a Printer01 (defined in t000.cpp)
printer = tutorial.Printer01()

#Create a Plasm, the graph structure of ecto
plasm = ecto.Plasm()

#insert our instance into the graph so that it may be executed
plasm.insert(printer)

#execute in a tight loop 10 times
plasm.execute(10)
