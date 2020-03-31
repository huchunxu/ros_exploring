#!/usr/bin/env python
import ecto #this must be imported before other ecto based python modules
import tutorial #this is our ecto module

#allocate cells
printer = tutorial.Printer02()
reader = tutorial.Reader01()

plasm = ecto.Plasm()
plasm.connect(reader["output"] >> printer["input"])

print "Press q,enter to quit. Enter text..."
#executes forever until a module returns a non zero value
while 0 == plasm.execute():
    pass
