.. _serialization:

.. highlight:: ectosh

Pure C++: serialization is awesome but I REALLY want to use pure C++
====================================================================

The core of ``ecto`` is pure C++ (the cells, the module creation, the plasm creation and the scheduler) so
doing everything in C++ is possible. Let just remind you of a few things:

  * graph based computation is far from being new and if you want pure C++ from A to Z, there are some great solutions
    out there but they might not give you all the ``ecto`` goodies (statistics, shell, ROS/PCL/OpenCV bindings, GUIS's,
    Python interfaces ... oh wait, you don't want that :) )
  * Python is part of the specs so that you can use any great code you find from C, C++ and Python by wrapping it in a cell
  * Python is used technically so that ``ecto`` does not reimplement module finding/loading
  * Python is used technically so that ``ecto`` does not reimplement some graph generation library and it can 
    actually be used as a pre-processing step (cf. :ref:`serialization`)
  * Python is only used as a glue so you don't get any performance hit: code still runs as fast as your cells
  * You can still set break points in your cells just like you would with pure C++ (cf :ref:`debugging`). The only bad
    thing is the ugliness of the stack (10-20 more levels as you have to go through the Python interpreter first but you don't need to look into it.
  * there is some overhead to learn Python but with ``ecto``, you just need to know how to create a plasm (cf. :ref:`tutorial-hello-plasms`)
    and that's the same overhead you would have to learn a graph generation syntax

If we are wrong about any of the above, please let us know (cf. contacts on the front page) !
If we are not, but you still don't want to use Python, we would like to know
why too, to see if we can fix anything (more docs/tutorials ?).

Anyhow, how do we code in pure C++ ? It depends on whether you want to dynamically
load your modules or include headers defining your cells.


Dynamic Loading
---------------

For now, ``ecto`` uses ``Boost Python`` to load modules and this is pure C++. There are other alternatives
like ``pluginlib`` as used in ``ROS`` but ``Boost Python`` was chosen for its maturity and widespread use.

First, use the ``ECTO_DEFINE_MODULE`` and ``ECTO_CELL`` macros to register your cells in ``Boost Python`` modules and add those
to your ``PYTHONPATH`` environment variables.

.. code-block:: cpp

  #include <ecto/python.hpp> //import this

  //inside a function called at the start of your program:
  //...
  //initialize python so that tests can use the registry
  Py_Initialize();
  //import the module of name module_name
  //be sure that the python path is correct.
  boost::python::import("module_path");
  // Create your cell of instance "SomeCell" in the ecto module "module_path"
  ecto::cell::ptr some_cell_ptr = ecto::registry::create("module_path::SomeCell");

The rest of the code to use your cells in plasms with a scheduler is describe in the section below.

Reference:
https://groups.google.com/a/plasmodic.org/forum/?fromgroups=#!topic/ecto-dev/sGXEv-svTD8


Non-Dynamic Loading
-------------------

The easiest is to look at some examples in the ``test`` folder. Those are self contained but you could define
the cells in their own headers. The notation in C++ is almost the same as in Python and a commented
example should be enough:

.. literalinclude:: ../../../../test/cpp/graph.cpp
   :language: cpp
