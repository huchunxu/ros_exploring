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
from ecto import _cell_base

class Cell(_cell_base):
    """
    When creating a cell from Python, just inherit from that class and define
    the same functions as in C++ if you want (i.e. declare_params(self, p),
    declare_io(self, p, i, o), configure(self, p, i, o) and process(self, i, o)
    """
    __looks_like_a_cell__ = True
    def __getattr__(self, name):
        if name == '__impl':
            return self
        else:
            if name in self.__dict__:
                return self.__dict__[name]
            else:
                raise AttributeError(self, name)

    def __init__(self, *args, **kwargs):
        _cell_base.__init__(self)
        if args:
            _cell_base.name(self, args[0])

        _cell_base.declare_params(self)

        for k, v in kwargs.iteritems():
            self.params.at(k).set(v)
        self.params.notify()
        _cell_base.declare_io(self)
        if self.__doc__ is None:
            self.__doc__ = "TODO docstr me."
        self._short_doc = self.__doc__
        self.__doc__ = self.gen_doc(self.__doc__)

    def short_doc(self):
        return self._short_doc

    @classmethod
    def inspect(cls, *args, **kwargs):
        m = cls(*args, **kwargs)
        return m
