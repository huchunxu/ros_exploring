Wrapping enumerations
=====================

The problem
-----------

Ecto cells that take parameters that are enumerations should **wrap
those enumerations using boost::python**, this vastly improves
use/readability.

For instance, this cell takes its parameter ``strategy`` as an integer:

.. highlight:: cpp

.. literalinclude:: ../../src/EnumAsInt.cpp

Which is suboptimal in several ways.  First, the documentation doesn't
say what the legal values are for this parameter:

.. ectocell:: ecto.ecto_examples EnumAsInt

And though the cell functions correctly if used correctly, (note here
that the direct call to ``process()`` is only for convenience, you'll
typically be adding the cell to a plasm and letting a scheduler do
this),

.. literalinclude:: ../../src/enumasint_okay.py

output:

.. program-output:: ../../src/enumasint_okay.py
   :in_srcdir:

it is easy to pass invalid values to the cell from python, and easy to
forget to handle out-of-range values:

.. literalinclude:: ../../src/enumasint_bad.py

output: 

.. program-output:: ../../src/enumasint_bad.py
   :in_srcdir:

Solution: enum parameters as enums, and wrap your enumerations
--------------------------------------------------------------

Make sure that the parameters are of the enum type itself.  In the
cell, declare and get the parameter as being of the enum type,

.. literalinclude:: ../../src/EnumAsEnum.cpp

And wrap the enum in the :ref:`ECTO_DEFINE_MODULE <ecto_define_module>` section like so:

.. literalinclude:: ../../src/module.cpp

Of course, this enumeration should be in a header file so that you
don't have multiple copies floating around.  Now a great many things
will be more usable.  First of all, the documentation for the module
will show the correct type (and therefore values) for the parameter:

.. ectocell:: ecto.ecto_examples EnumAsEnum

And if you try to pass magic numbers to cells, you get a reasonable error:

.. highlight:: py

.. literalinclude:: ../../src/enumasenum_bad.py

output:

.. highlight:: ectosh

.. program-output:: ../../src/enumasenum_bad.py
   :in_srcdir:
   :expect_error:
   :prompt:

The enum type gets a reasonable docstring,

.. program-output:: /usr/bin/env python -c 'import ecto.ecto_examples ; help(ecto.ecto_examples.Strategy)'
   :until: Methods inherited from

And the normal way to use this kind of thing would be:

.. highlight:: py

.. literalinclude:: ../../src/enumasenum_okay.py

.. highlight:: ectosh

.. program-output:: ../../src/enumasenum_okay.py
   :in_srcdir:


