.. _tendril-connections:

Tendril Connections
===================

.. highlight:: python

``>>`` is a cute little operator in python, and ecto abuses it
to make connections between cells....

Looking at a little interactive prompt:

  >>> from ecto.ecto_test import Increment, Add
  >>> i = Increment()
  >>> add = Add()
  >>> i['out']
   <ecto.TendrilSpecifications object at 0x20800d0>
  
So the the operator ``[]`` returns this special type, which we
won't talk about much but this special type represents the cell,
the input or output tendril and a few other properties. The key in
the ``[]`` operator will look up any input, or output in the
cell that matches it.

For example, if we give the wrong key:

  >>> add['a']
  RuntimeError: The module ecto_test::Add does not contain any inputs or outputs or parameters by the given name: a

Ok, so with this special object, ecto overrides the ``__rshift__``
python function.  This makes the following possible:

  >>> i['out'] >> add['right']
  [(<ecto_test.Increment object at 0x2062158>,
  'out',
  <ecto_test.Add object at 0x205ee68>,
  'right')]

So when the ``>>`` operator is used with two ``TendrilSpecifications`` objects, it returns a list of tuples, that descript
the connection.  Think of each tuple as containing ``(from cell, from port name, to cell, to port name)``.

Slice notation
--------------

You may notice the slice notation being used:

  >>> i[:] >> add['right']
  
This is shorthand for asking for all the outputs, or all the inputs of the cell.
You must be careful with this operation, as there must be a one to one mapping from the
left to right.  If all the inputs on the right have the same name as those on the left,
``i[:] >> add[:]``
would work. However, since ``i[:]`` is of "length" 1 and ``add[:]`` is of length 2, this
fails.

  >>> i[:] >> add[:]
  RuntimeError: Specification mismatch... len(lhs) != len(rhs) -> 1 != 2

Think all inputs to all outputs for the above statement.

Multicasting
------------

You may, in one line connect one output to many inputs in the following way:

  >>> i['out'] >> (add['left'],add['right])
  [(<ecto_test.Increment object at 0x2062158>,
  'out',
  <ecto_test.Add object at 0x205ee68>,
  'left'),
  (<ecto_test.Increment object at 0x2062158>,
  'out',
  <ecto_test.Add object at 0x205ee68>,
  'right')]

Notice the two tuples in the list now, each one describing a single connection from
an output tendril to an input tendril.


Plasms
------

Plasms have the :py:func:`ecto.Plasm.connect` which will take a list of connections
like the ones described above:

  >>> plasm = ecto.Plasm()
  >>> plasm.connect( i['out'] >> add['left'],
                     i2['out'] >> add['right'],
                     add['out'] >> printer['input'],
                    )

It is useful to always have a trailing ``,`` when writing the connection,
so that it is easy to append new connections to the function call.

You may also pass a vanilla python list to this function:

  >>> graph = []
  >>> graph += [ i['out'] >> add['left'],
                 i2['out'] >> add['right'],
                 add['out'] >> printer['input'],
               ]
  >>> plasm = ecto.Plasm()
  >>> plasm.connect(graph)
