.. _tutorial-hello-ecto:

Hello Ecto!
===========

The beginning. Well, you should have read :ref:`client-usage`
first. Your ecto experience will involve a hybrid approach to development, using
c++ and Python.

CMake
-----

You'll need a CMake file named ``CMakeLists.txt`` :download:`srcs/CMakeListsHello.txt`

.. literalinclude:: srcs/CMakeListsHello.txt
     :language: cmake

c++
---

So, the c++ side of things.  Ecto code structurally break down into
:ref:`Cells <cells-overview>` and :ref:`Modules <modules-overview>`

The module code:

  Download: :download:`srcs/tutorial.cpp`

  .. _code-module:

  .. literalinclude:: srcs/tutorial.cpp
     :language: cpp

The cell code:

  Download: :download:`srcs/Hello.cpp`

  .. _code-hello:

  .. literalinclude:: srcs/Hello.cpp
    :language: cpp

  Our first example.

As you can see in :ref:`Hello.cpp <code-hello>` this is one of the most basic ecto cells.

python
------
The python counter part to hello might look like:

.. _code-hello-python:

Download: :download:`srcs/hello.py`

.. literalinclude:: srcs/hello.py
  :language: python

The ``Ecto`` module is imported just like any Python module so make sure you have an ``__init__.py`` in the same folder.

Let's run it
------------

Well first, you need to build your module and your cell. If you put the 4 files above in a common folder,
(and make sure you renamed ``CMakeListsHello.txt`` to ``CMakeLists.txt``), just type:

.. code-block:: bash

   mkdir build && cd build && cmake ../ && make && cd ..

Now, ``ecto`` created a Python module so it needs to be added to your Python path:

.. code-block:: bash

   export PYTHONPATH=`pwd`/build:$PYTHONPATH

Also, do not forget this standard Python: if you have a folder containing Python modules, for it to become a proper package,
you need a file named ``__init__.py`` in it so:

.. code-block:: bash

   touch ./build/ecto_tutorial/__init__.py

Now, make your script executable and run it:

.. code-block:: bash

   chmod 755 hello.py && ./hello.py

.. program-output:: srcs/hello_doc.py
   :in_srcdir:
