.. _strands:

Strands:  preventing cells from running concurrently
====================================================

Strands are used by the schedulers when cells must not run
concurrently with specific instances or other cells.  Simply create a
strand and pass it as the argument ``strand`` when creating cells: no
cells that have the same strand will ever run at the same time.
 
One can also mark all instances of the same **type** of cell as always
thread-unsafe, see :ref:`ECTO_THREAD_UNSAFE() <ECTO_THREAD_UNSAFE>`.

Sample:

.. literalinclude:: strands.py

.. ectoplot:: strands.py plasm

.. program-output:: strands.py
   :in_srcdir:
