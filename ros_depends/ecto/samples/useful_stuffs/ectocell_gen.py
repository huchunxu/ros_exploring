#!/usr/bin/env python
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

import ecto
import os
import pkgutil

ecto_cell_template ='''
.. ectocell:: %(submodule_name)s %(cell_name)s
'''

usage = """ Generates an sphinx rst for that uses the ecto sphinx extension.

ectocell_gen.py [my_ecto_module] > my_cells.rst

where my_ecto_module is in the PYTHONPATH and is a ecto python module.
"""
if __name__ == '__main__':
    if len(os.sys.argv) == 2:
        module_name = os.sys.argv[1]
        module = __import__(module_name)
        all_modules = [(module_name,module)]
        for loader, submodule_name, is_pkg in  pkgutil.walk_packages(module.__path__):
            #print loader,submodule_name,is_pkg
            module = loader.find_module(submodule_name).load_module(submodule_name)
            all_modules.append((submodule_name,module))
        
        for submodule_name,module in all_modules:
            if submodule_name != module_name:
                submodule_name = '.'.join([module_name,submodule_name])
                heading = '-'
            else:
                heading ='='
            print '.. _'+submodule_name +':\n' #reference link.
            print submodule_name +'\n' + heading*len(submodule_name) + '\n'
            ecto_cells = ecto.list_all_ecto_modules(module)
            for cell in ecto_cells:
                cell_name = cell.__class__.__name__
                print ecto_cell_template%locals()
    else :
        print usage

