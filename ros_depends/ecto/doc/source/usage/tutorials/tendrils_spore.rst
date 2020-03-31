Tendrils and Spores
===================

:ref:`Tendrils <tendrils-overview>` are what defines the inputs/outputs/parameters of any cell.
They are a mapping from a string to the matching :ref:`tendril <tendril-overview>`. So what is a
tendril then ? Well, it is a container that holdes the input/output/parameter in a type-agnostic
way (like a ``boost::any``) to allow for static introspection and Python interaction.

We have seen some code that uses tendrils to extract the information at runtime but there are other ways of getting the data. Here we will show three different ways to get the data, but you will most likely always use the last one. The first method is the most convenient to write but the other methods can also have their use case (usually for lower level tendril manipulation).

We have seen how to use tendrils with a runtime registration:

.. literalinclude:: ./src_tendril_spore/Add1.cpp
   :language: c++


In an effort to save duplicate declaration, you may register a member :ref:`spore <spore-overview>` at
tendril declaration time. A spore is basically a typed wrapper around a tendril that behaves just like a pointer.

.. literalinclude:: ./src_tendril_spore/Add2.cpp
   :language: c++

And you can actually register a spore to a tendril when you're declaring it, making things even shorter:

.. literalinclude:: ./src_tendril_spore/Add3.cpp
   :language: c++
