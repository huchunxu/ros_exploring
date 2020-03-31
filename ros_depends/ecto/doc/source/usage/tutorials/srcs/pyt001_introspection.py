#!/usr/bin/env python
import ecto #this must be imported before other ecto based python modules
import tutorial #this is our ecto module

#allocate cells
printer = tutorial.Printer02()
reader = tutorial.Reader01()

def print_tendrils(tendrils):
    for x in tendrils:
        print "name:",x.key()
        t = x.data()
        print "doc:",t.doc
        print "type:",t.type_name
        print "value:",t.val
        print "has_default:", t.has_default
        print "user_supplied:", t.user_supplied
        print "required:",t.required
        print "dirty:",t.dirty

def inspect_cell(cell):
    print "Inspecting", cell.name()
    print 'Inputs:'
    print_tendrils(cell.inputs)
    print 'Outputs:'
    print_tendrils(cell.outputs)
    print 'Parameters:'
    print_tendrils(cell.params)

inspect_cell(reader)
inspect_cell(printer)

