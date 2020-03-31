.. _using-ecto:

Installing ecto
===============

If you have access to a ROS repository, just install the `ros-<distro>-ecto-xxx`
packages. E.g., if you are on indigo:

.. code-block:: bash

    sudo apt-get install ros-indigo-ecto-*

Otherwise, ecto is still easy to build from source:

.. toctree::
    :maxdepth: 1

    get_ecto.rst

To install with a cantankerous boost of your own choosing:

.. toctree::
    :maxdepth: 1

    non_system_boost.rst

Windows support is still experimental but we had good experience with ``cygwin``:

.. toctree::
    :maxdepth: 1

    get_ecto_win32
