.. highlight:: ectosh

.. _serialization:

Serialization: Create From Python, Load/Execute From Pure C++
=============================================================

.. _boost::serialization: http://www.boost.org/doc/libs/1_47_0/libs/serialization/doc/index.html

To allow such things as saving the state of a plasm, or network transport between
cells, ecto supports `boost::serialization`_ of tendrils,cells, and plasms.

Steps to ensure your types can be serialized:
  1. Write boost serialization methods for your types, don't worry all the basic types should work by default.
  2. Use the :c:macro:`ECTO_REGISTER_SERIALIZERS` in one of your translation units.

.. c:macro:: ECTO_REGISTER_SERIALIZERS(Type)


Sample:


.. code-block:: c++

  #include <ecto/ecto.hpp>
  struct Point
  {
    float x,y,z,n;
    template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
      ar & x & y & z & n;
    }
  };
  
  ECTO_REGISTER_SERIALIZERS(Point);
  //... more ecto code.


Embedding ecto in a c++ program
----------------------------------------

Assuming you have serializers for all your types, you may reliably serialize
a graph from python and execute in a native cpp main.

Now that we have some ecto cells, like the ones in hello_ecto, we can write
a python script that serializes the plasm to a text file.

.. literalinclude:: serialize_plasm.py

Our graph is the following:

.. ectoplot::  serialize_plasm.py plasm

Then a cpp file that can execute our ``printy.plasm`` file might look like:

.. literalinclude:: native_plasm.cpp
  :language: cpp

The cmake for this looks like:

.. literalinclude:: CMakeLists.txt
  :language: cmake

Running command like the following::

  % ./serialize_plasm.py
  % export PATH=PATH_TO_BIN:$PATH
  % tutorial_native_plasm

Yields this output::

  ** cell listing
  hello_ecto::Printer
  hello_ecto::Printer
  hello_ecto::Reader
  h ello o q
  h
  h
  ello
  ello
  o
  o


