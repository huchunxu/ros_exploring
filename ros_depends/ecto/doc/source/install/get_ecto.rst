Ecto From Source
================

Ecto is built using `catkin`_. Although it is the build tool favoured to build ROS packages, it is
also perfectly usable for a large suite of interdependent non-ros packages. It is in large part a
cmake/setuptools wrapper, so familiarity with these tools makes the jump to catkin not so large.

.. _`catkin`: http://docs.ros.org/api/catkin/html/

The instructions below assume an ubuntu system and use of the fundamental catkin tools.

Dependencies
------------

On ubuntu its simple....

.. code-block:: sh

    sudo apt-get install libboost-python-dev libboost-filesystem-dev libboost-system-dev \
            libboost-thread-dev python-setuptools python-gobject python-gtk2 graphviz doxygen \
            python-sphinx

For catkin and it's tools :

.. code-block:: sh

   sudo apt-get install python-catkin-pkg ros-indigo-catkin

Building
--------


.. code-block:: bash

  source /opt/ros/indigo/setup.bash
  mkdir -p ecto_ws/src
  cd ecto_ws/src
  catkin_init_workspace
  git clone http://github.com/plasmodic/ecto.git
  cd ..
  catkin_make

Running
-------

Now you have a working build of ecto! You should try to run a test.

.. code-block:: sh

    cd ecto_ws
    # add ecto to your python path
    . devel/setup.bash
    python src/ecto/samples/hello.py

You should see the following outputish:

::

    digraph G {
    graph [rankdir=TB, ranksep=1]
    edge [labelfontsize=8]
    0[label="hello_ecto::Reader"];
    1[label="hello_ecto::Printer"];
    2[label="hello_ecto::Printer"];
    0->1 [headlabel="str" taillabel="output"];
    0->2 [headlabel="str" taillabel="output"];
    }

    Enter input, q to quit
    hello there ecto q
    hello
    hello
    there
    there
    ecto
    ecto
    q
    q

Install
-------

You may install ecto using the following:

.. code-block:: sh

  cd build
  sudo make install
  sudo ldconfig


This will install ecto to the appropriate system paths. If built with catkin_make, it wil by default put
the resulting install space in `ecto_ws/install`. You can modify `CMAKE_INSTALL_PREFIX`
to redirect it:

.. code-block:: sh

  cd ecto_ws
  rm -rf build devel
  catkin_make -DCMAKE_INSTALL_PREFIX=/usr/local

Ecto deliverables get installed in the following locations:

.. code-block:: sh

  CMAKE_INSTALL_PREFIX/include/ecto-VERSION/
  CMAKE_INSTALL_PREFIX/share/ecto-VERSION/
  CMAKE_INSTALL_PREFIX/lib/python*/dist-packages/

Docs
----

Docs may be generated from the source in the following manner.

.. code-block:: sh

	cd build
	make sphinx-doc # for sphinx (prefer this for usage docs)
	make doxygen    # for c++ api docs
	ccmake .        # edit doc options.

Tests
-----

.. code-block:: sh

	cd build
	make test

or

.. code-block:: sh

	cd build
	ctest -V


Building Additional Ecto Repos
------------------------------

Build up your workspace with additional repos. Make sure system dependencies are installed (manually or via rosdep)
before building. Some other officially supported ecto repositories include:

* git clone http://github.com/plasmodic/ecto_image_pipeline.git
* git clone http://github.com/plasmodic/ecto_openni.git
* git clone http://github.com/wg-perception/opencv_candidate.git
* git clone http://github.com/plasmodic/ecto_opencv.git
* git clone http://github.com/plasmodic/ecto_pcl.git
* git clone http://github.com/plasmodic/ecto_ros.git

Alternative Build - Pure Catkin/CMake Style
-------------------------------------------

The preceding instructions utilise the typical tools made available through ROS. You can instead prefer
an almost pure catkin/cmake approach (sans the `ros-indigo-catkin` deb):


.. code-block:: bash
   
   mkdir -p ecto_ws/src && cd ecto_ws/src
   git clone http://github.com/ros/catkin.git
   git clone http://github.com/plasmodic/ecto.git
   ln -s catkin/cmake/toplevel.cmake CMakeLists.txt
   cd ..
   mkdir build
   cd build
   cmake ../src
   make -j5
   make install
   # source the environment, get ready to run!
   source devel/setup.bash

