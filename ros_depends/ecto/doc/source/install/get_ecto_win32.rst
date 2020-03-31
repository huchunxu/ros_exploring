.. _get-ecto-win32:

Requirements
============
 * boost (try 1.43 from boostpro http://www.boostpro.com/download/)
 * python (try 2.6)
 * cmake
 * gtest (build and add to path)
 * git (Follow http://help.github.com/win-set-up-git/)
 * msvc 2010 express ( http://www.microsoft.com/visualstudio/en-us/products/2010-editions/express )

path
=====
Windows needs some path setup for this to go smoothly.

.. code-block:: sh

    set PATH=C:\Program Files\CMake 2.8\bin;C:\Python26\;C:\gtest\lib_dir

Also it may be useful to always work from the **Visual Studio Command Prompt** or otherwise
be sure to source the ``C:\Program Files\Microsoft Visual Studio 10.0\VC\vcvarsall.bat`` before
starting any work.

http://technet.microsoft.com/en-us/sysinternals/bb896645

