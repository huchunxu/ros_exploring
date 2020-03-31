Python Binding Autoregistration
===============================

You may autoregister python bindings for modules.  This makes it
possible to store each cell's definition in a different translation
unit, reducing recompile times.

.. c:macro:: ECTO_CELL(pymodule_name, cell_type_name, "CellTypeName", "Cell Docstring")

Mark each ecto cell as such with the ``ECTO_CELL`` macro

.. code-block:: c++

  struct Add {
    int process(const ecto::tendrils& inputs, ...)
    // ...
  };

  ECTO_CELL(ecto_test, Add, "Add", "Add two doubles");

Where the arguments are 1. Python module name, but not as a
string, 2. Type of cell, 3.  string name of cell, 4. docstring for
cell.

This may be placed in any translation unit within the shared library,
e.g. in a file Add.cpp that contains the code above.  

.. _ecto_define_module:

.. c:macro:: ECTO_DEFINE_MODULE(pymodule_name)

In a single place inside the shared library (e.g. ``myecto_module.cpp``), call
``ECTO_DEFINE_MODULE``

.. code-block:: c++

  #include <ecto/ecto.hpp>
 
  ECTO_DEFINE_MODULE(ecto_test)
  {
    // additional boost::python or ecto bindings go here
  }

For instance, if cells in the ecto module contain parameters that are
enumerations, you may want to wrap the enumerations here so that
parameter values passed from the script have readable symbolic names.
