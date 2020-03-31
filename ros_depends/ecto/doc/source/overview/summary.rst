.. _checlist:

Quickguide
==========

You definitely need to go through more docs but this is the rough outline of what happens when you write ecto code: you define
Python/C++ cells that belong to a Python module. You then use that module to create a graph called ``plasm``, and you execute it.

Ecto can be both pure C++ or pure Python but we'll show you a combination of both as it provides the speed/tightness of the first one
and the flexibility of the second one.

You need to go through the following steps:

    * If you create C++ cells, you need to define your ecto module through an ``ectomodule`` CMake macro:

      .. code-block:: cmake

          ectomodule(my_ecto_module_name INSTALL
                                         DESTINATION ./here/different_name
                                         module.cpp
                                         awesome_file1.cpp
                                         awesome_file2.cpp
                     )
    * You need a module.cpp file that simply defines the Python module containing your cells

      .. code-block:: c++

          #include <ecto/ecto.hpp>
          ECTO_DEFINE_MODULE(my_module) { }

    * each cell needs to define 4 C++ functions

      .. code-block:: c++

          static void
          declare_params(tendrils&)
          static void
          declare_io(const tendrils&, tendrils&, tendrils&)
          void
          configure(const tendrils&, const tendrils&, const tendrils&)
          int
          process(const tendrils&, const tendrils&)

    * you create a Python script to create your graph called plasm and run it:

      .. code-block:: python

          #!/usr/bin/env python
          import ecto
          import my_awesome_cpp_ecto_module
          import my_awesome_python_ecto_module

          # create a plasm
          plasm = ecto.Plasm()

          # create some cells
          cell1 = my_awesome_cpp_ecto_module.MyAwesomeCell1(param1=whatever1)
          cell2 = my_awesome_python_ecto_module.MyAwesomeCell2(param2=whatever2)

          # connect those cells in the plasm
          plasm.connect(cell1['output'] >> cell2['input'])

          # execute the graph
          plasm.execute(niter=2)

Well, that seems like a lot of stuff to learn, really, what do I gain ? Well, let's see from the above:

    * you create a graph and execute it without recompiling code
    * your mix C++ and Python transparently
    * you have access to multithreading and scheduling transparently

Pretty good for a first start but ``ecto`` offers way, way more like:

    * more complex graphs than DAGs (like condition/for loops)
    * static introspection of cells
    * introspection of cell inputs/outputs during execution
    * runtime parameter change
    * serialization
    * pure C++ or pure Python code
    * GUI's
    * ...

Keep on reading ...
