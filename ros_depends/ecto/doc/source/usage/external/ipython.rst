.. _ipython:

Using IPython
=============

Provided your plasm doesn't use things like ecto_opencv.imshow or
other cells that create strange threads that conflict with ipython,
you can use IPython, http://ipython.scipy.org, as a front end
to ecto scripts.  In short, you execute the plasm asynchronously and
then spawn an ipython interpreter in the main python interpreter
thread:

.. literalinclude:: ipy.py

You should notice immediately that the console output from the
underlying C++ code (the Printer cell) conflicts with the ipython
prompt::

  % ../doc/source/ipy.py
  Threadpool executing in 3 threads.
  
  
  In [1]: ***** 1 ***** 0x7f96c8004a10
  ***** 2 ***** 0x7f96c8004a10
  ***** 3 ***** 0x7f96c8004a10
  ***** 4 ***** 0x7f96c8004a10
  ***** 5 ***** 0x7f96c8004a10
  ***** 6 ***** 0x7f96c8004a10
  
Ecto provides a function that will redirect the cout and cerr of the
ecto cells to a file, reducing the chance that something will
interfere with your ipython prompt:

.. literalinclude:: ipy_log.py

And you will see the output is separated.  Try the utility ``tail`` to
watch the output file: You can then change parameters of running cells
in the ipython session while the scheduler is executing.

Also see :ref:`argparsing` for enabling this in the command line in your
scripts.

.. autofunction:: ecto.log_to_file

.. autofunction:: ecto.unlog_to_file


  

