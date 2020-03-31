
.. highlight:: ectosh

.. _client-usage:

Using ecto in your own projects
===============================


CMake is the recommended build system tool.  For the purpose of this
documentation you should have a simple CMake project that looks like this on disk.

::

  % ls my_ecto_project
  CMakeLists.txt
  hello_ecto.cpp
  hello.py


CMake Setup
-----------

CMake should be used to find ecto and bring in a few macros:

.. code-block:: cmake

    cmake_minimum_required(VERSION 2.8)

    project(my_ecto_project)

    #defines include directories, libraries a few macros
    #that simplify creating ecto module targets.
    find_package(ecto REQUIRED)

    #make all libraries appear in the build/lib directory
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

    ectomodule(hello_ecto FALSE
        hello_ecto.cpp
    )

    #optionally link against other libs
    #link_ecto(hello_ecto
    #  ${MY_EXTRA_LIBS}
    #)

Take notice of the command ``find_package(ecto REQUIRED)``. If ecto is installed on your
system, or built somewhere, this will enable your project to become an ecto beast.

The ``ectomodule`` macro is the one that defines your C++ set of cells. The first argument is
mandotory and is the ``NAME`` of your module. It then accept a ``DESTINATION`` argument where you define
in which subfolder of your project to install the module and a ``INSTALL`` flag that defines whether your
module will be isntalled or not when during ``make install`` (you might not want to if it is a test module).
In case you want more flexibility, just know that this macro defines a ``${NAME}_ectomodule`` CMake target.

The second and last ecto macro you need to know is ``link_ecto`` that you can use to link your module to
external libraries (it just abstracts this ``${NAME}_ectomodule`` target for you).

ecto is installed
^^^^^^^^^^^^^^^^^

CMake should complete without error, using default settings.

::

    % cd my_ecto_project
    % mkdir build
    % cd build
    % cmake ..

Make sure that the output does not contain warnings or errors:

::

    % cmake ..
    -- The C compiler identification is GNU
    -- The CXX compiler identification is GNU
    -- Check for working C compiler: /usr/bin/gcc
    -- Check for working C compiler: /usr/bin/gcc -- works
    ... etc etc
    -- Configuring done
    -- Generating done

ecto is not installed
^^^^^^^^^^^^^^^^^^^^^

You will need to supply the ecto build path to your cmake cache.

::

    % cd my_ecto_project
    % mkdir build
    % cd build
    % cmake -Decto_DIR=~/ecto_dev/ecto/build ..

Make sure that the output does not contain warnings or errors:

::

    % cmake ..
    -- The C compiler identification is GNU
    -- The CXX compiler identification is GNU
    -- Check for working C compiler: /usr/bin/gcc
    -- Check for working C compiler: /usr/bin/gcc -- works
    ... etc etc
    -- Configuring done
    -- Generating done

Build it.
^^^^^^^^^

::

    % cd my_ecto_project/build
    % make

