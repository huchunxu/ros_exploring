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
'''
        Created on Mar 12, 2011

        @author: ethan.rublee@gmail.com (Ethan Rublee)
'''
import ecto
import sys
import inspect
from pydoc import ispackage
from inspect import ismodule
import multiprocessing

def print_tendrils(tendril, n):
    for x in tendril :
        #print "here"
        value = str(x.data().get())
        print  " - " + x.key() + " [%s]" % x.data().type_name + " default = %s" % value
        print  ""
        docstr = str(x.data().doc)
        doclines = docstr.splitlines()
        if doclines :
            for docline in doclines:
                print  "    " + docline
        print  ""

def print_module_doc(m):
    print m.__doc__

def list_all_cells(pymodule):
    '''
    Creates a list of all cells from a python module, which are ready for doc string and other
    types of introspection.
    '''
    l = []
    for x in dir(pymodule):
        mod = getattr(pymodule, x)
        if inspect.isclass(mod) and getattr(mod, '__looks_like_a_cell__', False):
            l.append(mod)
    return l
list_all_ecto_modules = list_all_cells

def list_cells(pymodule):
    l = []
    for x in dir(pymodule):
        mod = getattr(pymodule, x)
        if inspect.isclass(mod) and getattr(mod, '__looks_like_a_cell__', False):
            l.append(mod)
    return l
list_ecto_module = list_cells

def view_plasm(plasm, title=None):
    process = multiprocessing.Process(target=ecto.impl.view_plasm, args=(plasm,title))
    process.daemon = True
    process.start()
