#!/usr/bin/env python
import ecto #this must be imported before other ecto based python modules
import tutorial #this is our ecto module

#instantiation
printer = tutorial.Printer01()

#setup a single node graph
plasm = ecto.Plasm()
plasm.insert(printer)

#introspection
print printer.__doc__
print plasm.viz()
#displays a graphiz rendering
ecto.view_plasm(plasm)

