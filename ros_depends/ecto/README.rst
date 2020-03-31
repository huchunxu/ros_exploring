Ecto
====
Ecto is a hybrid C++/Python development framework for constructing and maintaining
pipelines.  In Ecto, pipelines are constructed in terms of processing units, ``Cells``,
connected by data paths, ``Tendrils``, that form *Directed Acyclic Graphs*, ``Plasms``.
Cells are typically written in C++, tendrils may be any type, and the plasm may
be executed in a variety of clever ways. Python is uses as a the graph DSL.

Ecto may be found useful in domains such as perception, audio, or robotics.

To get started see the online docs at http://plasmodic.github.io/ecto/

Get and Build Ecto
==================
These instructions are useful if you wish to work with Ecto from source, as a
standalone library.

source
^^^^^^

We use git for our source control.  You can get a copy of our repo by doing the following::

   git clone git://github.com/plasmodic/ecto.git

dependencies
^^^^^^^^^^^^
Ecto requires

- CMake
   CMake is used for our build system, and you will need a version >= 2.8
- Boost
   Anything over 1.40 http://www.boost.org
- Python
   Ecto should work with 2.6 and up.  You should have the development libraries.
   If you are bellow 2.7 you should install the argparse library
- *optional* Sphinx
   Docs are built with sphinx, >= v1.0.7
- *optional* gtest
   http://code.google.com/p/googletest

On ubuntu you can get most of these through apt::

   sudo apt-get install cmake libboost-all-dev python-dev python-argparse python-yaml libgtest-dev

To build the docs, you should use a very recent version of Sphinx::

   sudo easy_install -U sphinx

build
^^^^^
To build you should just follow a normal cmake recipe::

   cd ecto
   mkdir -p build
   cd build
   cmake ..
   make

test
^^^^
To validate ecto using our test suite, you may::

   cd ecto/build
   make
   ctest

This should report zero test errors. If it does report an error, please tell us about it
here: https://github.com/plasmodic/ecto/issues/new


docs
^^^^
To create the latest documentation for Ecto::

   sudo pip install -U catkin_sphinx
   sphinx-build -b html ./doc/source/ ./doc/build

To build documentation for the kitchen:
::

  sphinx-build -b html ./doc/kitchen/ ./doc/build/

Then you can open up ``ecto/build/doc/html/index.html`` locally.

install
^^^^^^^
To install Ecto on your machine::

   cd ecto/build
   make install

use
^^^
See the documentation (http://plasmodic.github.io/ecto/) for detailed usage instructions.
