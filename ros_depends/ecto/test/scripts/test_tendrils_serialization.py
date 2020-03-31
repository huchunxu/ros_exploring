#!/usr/bin/env python
#
# Copyright (c) 2012, Industrial Perception, Inc.
# All rights reserved.
#
import ecto
from ecto import ecto_test
import math
from cStringIO import StringIO
def test_tendrils_serialization():
    t = ecto.Tendrils()
    t.declare("Hello",ecto.Tendril.createT('std::string'))

    t.declare("Woot",ecto.Tendril.createT('double'))
    t.Hello = 'World'
    t.Woot = math.pi
    ofs = StringIO()
    t.save(ecto.ostream(ofs))

    #grab the string
    ifs = StringIO(ofs.getvalue())

    t_ds = ecto.Tendrils()
    t_ds.load(ecto.istream(ifs))
    print 'loaded:'
    for key, val in t_ds.iteritems():
        print key, val.val
    assert t_ds.Woot == math.pi
    assert t_ds.Hello == 'World'
if __name__ == '__main__':
    test_tendrils_serialization()

