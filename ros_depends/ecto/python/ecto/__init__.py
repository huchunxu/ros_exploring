#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import platform, sys
from ecto.ecto_main import _cell_base, _cell_cpp, __getitem_list__, __getitem_slice__, __getitem_str__, __getitem_tuple__, lookup
from . import impl

#import inspect

class EctoCellBase(object):
    pass

def cell_getitem(self, *args, **kwargs):
    if len(args) == 1 and type(args[0]) == slice:
        return __getitem_slice__(self.__impl, args[0])
    if len(args) == 1 and type(args[0]) == tuple:
        return __getitem_tuple__(self.__impl, args[0])
    if len(args) == 1 and type(args[0]) == list:
        return __getitem_list__(self.__impl, args[0])

    return __getitem_str__(self.__impl, args[0])

def cellinit(cpptype):
    def impl(self, *args, **kwargs):
        if len(args) > 1:
            raise RuntimeError("Too many positional args:  only one allowed, representing cell instance name")
        e = lookup(cpptype)
        c = self.__impl = e.construct()
        if len(args) == 1:
            self.__impl.name(args[0])
        e.declare_params(self.__impl.params)
        # c.construct(args, kwargs)
        #print "c=", c
        self.inputs = c.inputs
        self.outputs = c.outputs
        self.params = c.params
        for k, v in kwargs.iteritems():
            if k == 'strand':
                self.__impl._set_strand(v)
            elif k == "connected_inputs_only":
                self.__impl._set_process_connected_inputs_only(v)
            elif isinstance(v, _cell_cpp):
                setattr(self.params, k, v.__impl)
            else:
                setattr(self.params, k, v)
            # print "now:", getattr(self.params, k)
        e.declare_io(self.params, self.inputs, self.outputs)
        try:
            self.__impl.verify_params()
        except ecto.EctoException as e:
            print >>sys.stderr, cpptype
            raise type(e)('\nCell Type: %s\nCell Name: %s\nWhat:\n%s'%(cpptype,self.__impl.name(),str(e)))
    # self.params.get('k') = v
    return impl

def cell_print_tendrils(tendril):
    s = ""
    for x in tendril:
        try:
            value = str(x.data().get())
        except TypeError, e:
            value = "[unprintable]"
        s += " - " + x.key() + " [%s]" % x.data().type_name
        if x.data().required:
            s += " REQUIRED"

        if x.data().has_default:
            s += " default = " + value
        s += "\n"
        docstr = str(x.data().doc)
        doclines = docstr.splitlines()
        if doclines :
            for docline in doclines:
                s +=  "    " + docline + "\n"
        s +=  "\n"
    return s

@classmethod
def cell_inspect(self, *args, **kwargs):
    c = self.__factory()
    c.declare_params()
    c.declare_io()
    return c

def cell_process(self):
    return self.__impl.process()

def cell_configure(self):
    return self.__impl.configure()

def cell_activate(self):
    return self.__impl.activate()

def cell_deactivate(self):
    return self.__impl.deactivate()

def cell_name(self):
    return self.__impl.name()

def cell_typename(self):
    return self.__impl.typename()

def cell_doc(short_doc, c):
    doc =short_doc + "\n\n"
    params = cell_print_tendrils(c.params)
    inputs = cell_print_tendrils(c.inputs)
    outputs = cell_print_tendrils(c.outputs)
    if(params):
        doc += "Parameters:\n%s"%params
    if(inputs):
        doc += "Inputs:\n%s"%inputs
    if(outputs):
        doc += "Outputs:\n%s"%outputs
    return doc

def postregister(cellname, cpptypename, short_doc, inmodule):
    e = lookup(cpptypename)
    c = e.construct()
    c.declare_params()
    c.declare_io()
    thistype = type(cellname, (_cell_cpp,),
                    dict(__doc__ = cell_doc(short_doc,c),
                         __module__ = inmodule.__name__,
                         inputs = c.inputs,
                         outputs = c.outputs,
                         params = c.params,
                         type = c.typename,
                         short_doc = short_doc,
                         __init__ = cellinit(cpptypename),
                         __getitem__ = cell_getitem,
                         inspect = cell_inspect,
                         process = cell_process,
                         configure = cell_configure,
                         activate = cell_activate,
                         deactivate = cell_deactivate,
                         name = cell_name,
                         type_name = cell_typename,
                         __factory = e.construct,
                         __looks_like_a_cell__ = True
                         ))

    inmodule.__dict__[cellname] = thistype


#allow build/lib/ecto and src/ecto/python/ecto
#from pkgutil import extend_path
#__path__ = extend_path(__path__, __name__)

#load ecto.so
#from ecto.load_pybindings import load_pybindings
#load_pybindings(__name__)

from ecto.cells import *
from ecto.ecto_main import *

from doc import *
from cell import *
from blackbox import BlackBox, BlackBoxCellInfo, BlackBoxError, BlackBoxForward
from schedulers import MultiPlasmScheduler
import test


