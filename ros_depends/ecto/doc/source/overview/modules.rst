.. _modules-overview:

Modules
=======

An ecto module is a collection of cells, and manifests itself as a single shared
object.  From a build system perspective a module looks like:

.. code-block:: cmake
  
  ectomodule(my_ecto_module DESTINATION path_to_relative_build_module INSTALL
    module.cpp
    Hello.cpp
    SomeCell.cpp
    )

Where ``module.cpp`` is the recommended idiom for defining your ecto module.
INSTALL is an optional argument: if set, the module will also be installed (you might
not want to install a test module). DESTINATION is the relative path where to 
install the module (e.g.: ``./`` (default) or ``${PROJECT_NAME}/ecto_cells``).
It will contain very little code and use the :c:macro:`ECTO_DEFINE_MODULE`. A typical
``module.cpp`` will look like:

  .. code-block:: c++
    
    #include <ecto/ecto.hpp>
  
    ECTO_DEFINE_MODULE(my_ecto_module){}
  
  It is important that the name given to your ``ectomodule`` macro, ``my_ecto_module``
  matches that given to :c:macro:`ECTO_DEFINE_MODULE`.

And every other cpp file will contain typically a single cell, exported:

.. code-block:: c++
  
  #include <ecto/ecto.hpp>
  
  namespace my_ecto_module
  {
    struct Hello
    {
      //... cell implementation
    };
  }
  
  ECTO_CELL(my_ecto_module, my_ecto_module::Hello,"Hello", "Prints a string input to standard output.");
  

Note that namespacing your implementations is a good idea to avoid linktime One Definition Rule (ODR)
violations.  Also notice that most ecto modules do not contain header files, as your
ecto cells should be rather self contained, use common types for 
tendrils (int, float, cv::Mat, pcl::PointCloud), etc ...

The ``ectomodule`` cmake macro will yield a shared library artifact in your build, that
has no lib prefix. This shared library is accessible to python if it is in 
the python python path.

.. code-block:: python
  
  #import the ecto module
  import my_ecto_module
  
  #access a cell from the module
  hello = my_ecto_module.Hello()

Your project may contain any number of modules, and each module may have any number
of cells. More advanced projects might use python package layout, which should work
fine with compiled ecto modules, see :ref:`techniques-python-packages`.



