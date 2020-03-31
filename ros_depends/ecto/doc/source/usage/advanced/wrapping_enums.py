Debugging with gdb and valgrind
===============================

.. highlight:: ectosh

.. index:: gdb; running scripts under

Running an ecto script under gdb
--------------------------------

You'll notice that as such, you can't run a script under gdb::

  % gdb ../src/ecto/test/scripts/test_random.py                       
  GNU gdb (Ubuntu/Linaro 7.2-1ubuntu11) 7.2
  [ chattiness ]
  "/ssd/ecto_kitchen/src/ecto/test/scripts/test_random.py": not in executable format: File format not recognized

The trick is to run ``gdb`` on the python binary, not the script, and
pass the script as an argument.  ``gdb`` will detect when the script
imports modules of ecto cells and debug those too.  Use gdb's
``--args`` argument, for instance::

  % gdb --args /usr/bin/python ../ecto/test/scripts/test_random.py
  GNU gdb (Ubuntu/Linaro 7.2-1ubuntu11) 7.2
  Copyright (C) 2010 Free Software Foundation, Inc.
  License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
  This is free software: you are free to change and redistribute it.
  There is NO WARRANTY, to the extent permitted by law.  Type "show copying"
  and "show warranty" for details.
  This GDB was configured as "x86_64-linux-gnu".
  For bug reporting instructions, please see:
  <http://www.gnu.org/software/gdb/bugs/>...
  Reading symbols from /usr/bin/python...(no debugging symbols found)...done.
  (gdb) 

At this point, since you haven't run the script yet, gdb won't know
about the cells inside any of the compiled modules that the script
imports.  Run the script once to trigger the shared library loads::

  (gdb) r
  Starting program: /usr/bin/python ../ecto/test/scripts/test_random.py
  [Thread debugging using libthread_db enabled]
  ***** 0.294665 ***** 0x1031980
  ***** 0.181778 ***** 0x1031980
  [etc]  
  ***** 0.850141 ***** 0x1031980
  [Inferior 1 (process 24050) exited normally]
  (gdb) 

Now you can set breakpoints inside cells, don't forget any namespace
that the cell lives in::

  (gdb) break ecto_test::Uniform01::process(ecto::tendrils const&, ecto::tendrils&) 
  Breakpoint 1 at 0x7fffec5d6169: file /home/ecto_kitchen/ecto/test/modules/Uniform01.cpp, line 88.
  (gdb) r
  Starting program: /usr/bin/python ../ecto/test/scripts/test_random.py
  [Thread debugging using libthread_db enabled]
  
  Breakpoint 1, ecto_test::Uniform01::process (this=0x1031950, inputs=..., outputs=...)
      at /home/ecto_kitchen/ecto/test/modules/Uniform01.cpp:88
  88	      for (unsigned j=0; j<ncalls; ++j)
  (gdb) 
  
As usual ``where`` will get you your backtrace.  As you can see below,
as you go up the stack you pass from the cell's code (the ``process``
method)::

  (gdb) where
  #0  ecto_test::Uniform01::process (this=0x1031950, inputs=..., outputs=...)
    at /home/ecto_kitchen/ecto/test/modules/Uniform01.cpp:88

up through ecto's compiled C++ code::

  #1  0x00007fffec5dd322 in ecto::cell_<ecto_test::Uniform01>::process (this=0x971f00, inputs=..., outputs=...)
      at /home/ecto_kitchen/ecto/include/ecto/cell.hpp:366
  #2  0x00007fffec5dc270 in ecto::cell_<ecto_test::Uniform01>::dispatch_process (this=0x971f00, inputs=..., outputs=...)
      at /home/ecto_kitchen/ecto/include/ecto/cell.hpp:378
  #3  0x00007ffff55b0beb in ecto::cell::process (this=0x971f00) at /home/ecto_kitchen/ecto/src/lib/cell.cpp:137
  #4  0x00007ffff55e754a in ecto::schedulers::invoke_process (graph=..., vd=0)
      at /home/ecto_kitchen/ecto/src/lib/schedulers/invoke.cpp:39
  #5  0x00007ffff55e9402 in ecto::schedulers::singlethreaded::invoke_process (this=0x1031890, vd=0)
      at /home/ecto_kitchen/ecto/src/lib/schedulers/singlethreaded.cpp:56
  #6  0x00007ffff55e981b in ecto::schedulers::singlethreaded::execute_impl (this=0x1031890, niter=100)
      at /home/ecto_kitchen/ecto/src/lib/schedulers/singlethreaded.cpp:121
  #7  0x00007ffff55e9750 in ecto::schedulers::singlethreaded::execute (this=0x1031890, niter=100)
      at /home/ecto_kitchen/ecto/src/lib/schedulers/singlethreaded.cpp:102

through ecto's python binding layer::

  #8  0x00007ffff66acfdc in ecto::py::execute1<ecto::schedulers::singlethreaded> (s=..., arg1=100)
      at /home/ecto_kitchen/ecto/src/pybindings/schedulers.cpp:11
  #9  0x00007ffff66b84b6 in boost::python::detail::invoke<boost::python::to_python_value<int const&>, int (*)(ecto::schedulers::singlethreaded&, unsigned int), boost::python::arg_from_python<ecto::schedulers::singlethreaded&>, boost::python::arg_from_python<unsigned int> > (rc=..., f=@0x963cd8, ac0=..., ac1=...)
      at /usr/include/boost/python/detail/invoke.hpp:75
  #10 0x00007ffff66b69c8 in boost::python::detail::caller_arity<2u>::impl<int (*)(ecto::schedulers::singlethreaded&, unsigned int), boost::python::default_call_policies, boost::mpl::vector3<int, ecto::schedulers::singlethreaded&, unsigned int> >::operator() (this=0x963cd8, args_=0xf5d128) at /usr/include/boost/python/detail/caller.hpp:223
  #11 0x00007ffff66b53e3 in boost::python::objects::caller_py_function_impl<boost::python::detail::caller<int (*)(ecto::schedulers::singlethreaded&, unsigned int), boost::python::default_call_policies, boost::mpl::vector3<int, ecto::schedulers::singlethreaded&, unsigned int> > >::operator() (this=0x963cd0, args=0xf5d128, kw=0x1028320)
      at /usr/include/boost/python/object/py_function.hpp:38
  #12 0x00007ffff6178c2e in boost::python::objects::function::call(_object*, _object*) const ()
     from /usr/lib/libboost_python-py26.so.1.40.0
  #13 0x00007ffff6178ed8 in ?? () from /usr/lib/libboost_python-py26.so.1.40.0
  #14 0x00007ffff618054b in boost::python::handle_exception_impl(boost::function0<void>) ()
     from /usr/lib/libboost_python-py26.so.1.40.0
  #15 0x00007ffff61757d8 in ?? () from /usr/lib/libboost_python-py26.so.1.40.0

up to the python interpreter itself::

  #16 0x000000000041f0c7 in PyObject_Call ()
  #17 0x00000000004a7378 in PyEval_EvalFrameEx ()
  #18 0x00000000004a8550 in PyEval_EvalFrameEx ()
  #19 0x00000000004a9671 in PyEval_EvalCodeEx ()
  #20 0x00000000004a9742 in PyEval_EvalCode ()
  #21 0x00000000004c9a0e in PyRun_FileExFlags ()
  #22 0x00000000004c9c24 in PyRun_SimpleFileExFlags ()
  #23 0x000000000041a7ff in Py_Main ()
  #24 0x00007ffff69d8c4d in __libc_start_main () from /lib/libc.so.6
  #25 0x00000000004199f9 in _start ()
  

.. index:: Exceptions; catching under gdb

Exceptions
----------

If your problem is e.g. a segfault or a null pointer dereference, just
running the script under gdb will get you to the point of the fault.
If on the other hand something is throwing an exception, the program
will exit::

  (gdb) r
  Starting program: /usr/bin/python ../ecto/test/scripts/test_random.py
  [Thread debugging using libthread_db enabled]
  Traceback (most recent call last):
    File "../ecto/test/scripts/test_random.py", line 22, in <module>
      test_random()
    File "../ecto/test/scripts/test_random.py", line 16, in test_random
      sched.execute(niter=100)
  RuntimeError: Original Exception: std::runtime_error
    What   : catastrophe!
    Module : Random
    Function: process
  [Inferior 1 (process 11876) exited with code 01]
  (gdb) where
  No stack.

As you can see there is no helpful information here.  The trick is to
``catch throw``::

  (gdb) catch throw
  Catchpoint 2 (throw)
  (gdb) r
  Starting program: /usr/bin/python ../ecto/test/scripts/test_random.py
  [Thread debugging using libthread_db enabled]
  Catchpoint 2 (exception thrown), 0x00007ffff4f0fde0 in __cxa_throw () from /usr/lib/libstdc++.so.6
  (gdb) where
  #0  0x00007ffff4f0fde0 in __cxa_throw () from /usr/lib/libstdc++.so.6
  #1  0x00007fffeaa64707 in ecto_test::Uniform01::process (this=0x1033220, inputs=..., outputs=...)
      at /home/ecto_kitchen/ecto/test/modules/Uniform01.cpp:88
  #2  0x00007fffeaa6b330 in ecto::cell_<ecto_test::Uniform01>::process (this=0x9737b0, inputs=..., outputs=...)
      at /home/ecto_kitchen/ecto/include/ecto/cell.hpp:366
  #3  0x00007fffeaa6a27e in ecto::cell_<ecto_test::Uniform01>::dispatch_process (this=0x9737b0, inputs=..., outputs=...)
      at /home/ecto_kitchen/ecto/include/ecto/cell.hpp:378
  (gdb) up
  #1  0x00007fffeaa64707 in ecto_test::Uniform01::process (this=0x1033220, inputs=..., outputs=...)
      at /home/ecto_kitchen/ecto/test/modules/Uniform01.cpp:88
  88	      throw std::runtime_error("catastrophe!");
  (gdb) l
  83	      ncalls=parameters.get<unsigned>("ncalls");
  84	    }
  85	
  86	    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
  87	    {
  88	      throw std::runtime_error("catastrophe!");      // <-    boom!
  89	
  90	      for (unsigned j=0; j<ncalls; ++j)
  91	        *out_ = (*pimpl_)();
  92	      return ecto::OK;


.. index:: Valgrind, gdb

Valgrind
--------

You'll need to have a look at `what the Python devs say about this
<http://svn.python.org/projects/python/trunk/Misc/README.valgrind>`_... in
short, for performance reasons python deliberately does some things
that look suspicious to valgrind but in fact are not.  So you'll need
to use the suppressions file that comes with python, called
`valgrind-python.supp
<http://svn.python.org/projects/python/trunk/Misc/valgrind-python.supp>`_.


