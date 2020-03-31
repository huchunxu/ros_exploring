.. _black_box:

BlackBox
========

The ``BlackBox`` is an encapsulation method for ecto graphs: it behaves like a meta-cell.

Example
-------

Let's take the plasm from :ref:`tutorial-hello-plasms` but let's assume that we want to
encapsulate it so that it looks like a cell to a user. Let's use a ``BlackBox``.

.. literalinclude:: blackbox.py
  :language: py

Now, let's go over the example. We first create a class to encapsulate the plasm.

.. literalinclude:: blackbox.py
  :language: py
  :lines: 6-11

We know we will forward the parameters of ``i2`` to the user and the output of ``add``.
To have ``declare_io`` and ``declare_params`` work statically, we need to define those cells
in ``declare_cells``.

.. literalinclude:: blackbox.py
  :language: py
  :lines: 12-26

We also need to define the different forwards

.. literalinclude:: blackbox.py
  :language: py
  :lines: 27-43

We also need to define the cells like we would in C++ in ``configure`` except some are
already known as they were defined in ``declare_cells``.

.. literalinclude:: blackbox.py
  :language: py
  :lines: 44-48

Finally, we need to define the connections like we would in the plasm:

.. literalinclude:: blackbox.py
  :language: py
  :lines: 49-53

If we execute that ``BlackBox``, we get the same as when running the plasm:

.. program-output:: blackbox.py
   :in_srcdir:

Python definition
-----------------

Here is the full Python definition and what members you are supposed to override.

.. autoclass:: ecto.BlackBox
  :members:

More Examples
-------------

Here are a few more examples from the official ecto tests in *test/scripts/test_blackbox*:

.. literalinclude:: ../../../../test/scripts/test_blackbox.py
  :language: py
  :lines: 29-69, 123-154
