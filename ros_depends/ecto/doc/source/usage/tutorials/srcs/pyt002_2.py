#!/usr/bin/env python
import ecto #this must be imported before other ecto based python modules
import tutorial #this is our ecto module

#Notice the node names.
printer = tutorial.Printer03('Prefix printer',prefix='prefixture')
reader = tutorial.Reader01('stdin reader')

plasm = ecto.Plasm()
plasm.connect(reader["output"] >> printer["input"])

#Notice the difference in visualization
print plasm.viz()
ecto.view_plasm(plasm)

print "Press q,enter to quit. Enter text..."
#executes forever until a module returns a non zero value
while 0 == plasm.execute():
    pass
