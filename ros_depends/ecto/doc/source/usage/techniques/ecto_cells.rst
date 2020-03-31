.. _builtin_cells:

Builtin Ecto Cells
==================
Ecto contains a few cells that are generally useful for Ecto graph construction.

Convenience cells
-----------------
The are cells that may be generally useful in your ecto graphs and tend to be
weakly typed.

.. ectocell:: ecto Constant

   This cell takes a python parameter that becomes a constant
   output. As long as this python type is convertable to a c++ type
   (through ``python`` bindings or basic types), it may be connected
   to cells written in c++.

.. ectocell:: ecto Passthrough

   Passthrough is mostly useful from within a :ref:`black_box` to connect
   multiple inputs to one cell, which then becomes a singular input for the BlackBox.

.. _trueeveryn:

.. ectocell:: ecto TrueEveryN

.. _if:

.. ectocell:: ecto If

   The If cell enables you to conditionally execute a single cell
   based on an input flag.

   An example of using the If cell for condition execution of a
   cell. Notice that cell that is encased in an If is **not added to the
   graph**, it is passed to the ``If`` cell as a **parameter**.  

   .. literalinclude:: conditional.py

   The above sample uses the built in cell :ref:`TrueEveryN <trueeveryn>`
   
.. _entanglement:

Entanglement
------------

To support feedback loops, asynchronous execution of graphs, and
conditional execution, ecto supplies a concept of entangled cells,
which allow communication of values without breaking the acyclic or
synchronous nature of graph execution.

.. autofunction:: ecto.EntangledPair

  EntangledPair is useful when you would like to feed the output of
  one cell into the input of another cell, without creating a cycle in
  the graph. Keep in mind that the first execution of the graph will
  result in the default value of the whatever the source is connected
  to being used.

Here is an example of using the EntangledPair for feedback in a graph.

.. literalinclude:: feedback.py

The output is:

.. program-output:: feedback.py
   :in_srcdir:

And the graph looks like the following. Notice that there is no cycle:

.. ectoplot:: feedback.py plasm


Dealer
------------

It may be that you have some iterable in python that you would like to
pay out to your graph, one item at a time. Enter the Dealer, think blackjack.

.. autofunction:: ecto.Dealer

  The argument ``typer`` should be a tendril, which already has a type associated
  with it. This is used to prime the dealer's type erasure, so that boost python
  objects are converted immediately to this type.

Here is an example of using the Dealer.

.. literalinclude:: dealer.py

The output is:

.. program-output:: dealer.py
   :in_srcdir:

And the graph looks like the following. Notice that there is no cycle:

.. ectoplot:: dealer.py plasm
