.. _tutorial-lifetime:

Lifecycle of Cells
==================

This tutorial is more of a demo of what happens during the lifetime of a cell.
Please read the reference for a :ref:`cell <cells-overview>` for a more detailed reference.

This cell will chirp at every point in its life.

  Download: :download:`srcs/LifeCycle.cpp`

  .. literalinclude:: srcs/LifeCycle.cpp
     :language: cpp

Also here is a python script that will trigger the lifecycle exploration.

  Download: :download:`srcs/lifecycler.py`

  .. literalinclude:: srcs/lifecycler.py
     :language: python

Lets run it:

  .. program-output:: srcs/lifecycler.py
    :in_srcdir:
