.. _argparsing:

Command line helpers: getting stats, GUIS, a shell and more automagically !
===========================================================================

.. _argparse: http://docs.python.org/library/argparse.html

Requires the python `argparse`_ library.

Ecto has a few helper functions for commandline parsing using
the argparse library.  These are useful for having scripts that
may run with multiple ecto schedulers based on command line
args.

sample
------

A sample of using argparse with ecto.

.. literalinclude:: sample_opts.py
   :language: python

Running with ``--help`` gives us:

.. program-output:: sample_opts.py --help
    :in_srcdir:

ipython
-------
To use with ipython, simply pass ``--shell`` to the sample.  This will result
in the plasm being executed asynchronously, and pop you out into an
ipython shell.

.. highlight:: ectosh

::

  %  ./sample_opts.py --shell --niter=3
  ***** 15 ***** 0x14e1a90
  ***** 15 ***** 0x14e1a90
  ***** 15 ***** 0x14e1a90
  
  
  Input <1>c.outputs.out
   Out[1]: 3
  
  Input <2>m.outputs.out
   Out[2]: 15.0
  
  Input <3>c.outputs.out = 5
  
  Input <4>sched.execute(niter=2)
  ***** 25 ***** 0x14e1a90
  ***** 25 ***** 0x14e1a90
   Out[4]: 0

Notice how the shell has access to the local variables declared
in the script.

Exposing cells to the parser
----------------------------
You can also automatically expose any of the parameters in a Cell like object
to the argparse parser.

.. literalinclude:: sample_opts2.py
   :language: python

The help looks like:

.. program-output:: sample_opts2.py --help
    :in_srcdir:


.. autofunction:: ecto.opts.cell_options
   
   cell_options returns a cell factory. The cell factory can be used to produce
   an instance of a cell based on the arguments parsed.

helper functions
----------------

.. autofunction:: ecto.opts.use_ipython

  The variables options, sched, plasm, and all locals passed in will
  be available in the ipython context.

.. autofunction:: ecto.opts.run_plasm

.. autofunction:: ecto.opts.scheduler_options

.. autofunction:: ecto.opts.doit

