
.. index:: Valgrind, gdb
.. index:: profiling


.. highlight:: ectosh

Valgrind
========

Valgrind is our tool of choice for hunting nasty memory errors and/or
profiling. Then again, if you just want to compare the performance of your cells using
``callgrind``, ``ecto`` provides runtime statistics as explained in :ref:`argparsing`.

Suppressions
------------

First you'll need to have a quick look at `what the Python devs say
about this
<http://svn.python.org/projects/python/trunk/Misc/README.valgrind>`_... in
short, for performance reasons python deliberately does some things
that look suspicious to valgrind but in fact are not.  So you'll need
to use the suppressions file that comes with python, called
`valgrind-python.supp
<http://svn.python.org/projects/python/trunk/Misc/valgrind-python.supp>`_.
Download that via that link and **READ THE FILE**, there are some
suppressions you need to uncomment in there, if you don't intend to
rebuild python itself.


Running ecto under valgrind
---------------------------

You must run valgrind on the python interpreter... not
the script!  That is, not::

  valgrind --tool=memcheck --suppressions=valgrind-python.supp myscript.py

but::

  valgrind --tool=memcheck --suppressions=valgrind-python.supp python myscript.py

This is basically the same situation/mechanism as when running ecto
scripts under ``gdb``.



Profiling
---------

To find the performance bottlenecks in your application you'll
probably want to start with the graph that the threadpool scheduler
prints out at the end of each run, then narrow down your script to the
bits that take the longest time.  You can run under a Threadpool or
Singlethreaded scheduler, but valgrind will (on purpose) interfere
with your threads and ensure that things run essentially
singlethreaded, so the Singlethreaded scheduler probably makes more
sense: just less mechanics to go wrong.

I like to use some of callgrinds' nifty features to take a sample of a
plasm while it is running in a steady state, i.e. I start my plasm like this::

  % valgrind --tool=callgrind --instr-atstart=no python ./colorize_clusters.py
  ==5702== Callgrind, a call-graph generating cache profiler
  ==5702== Copyright (C) 2002-2010, and GNU GPL'd, by Josef Weidendorfer et al.
  ==5702== Using Valgrind-3.6.1 and LibVEX; rerun with -h for copyright info
  ==5702== Command: python ../src/pcl/samples/ros/colorize_clusters.py
  ==5702==
  ==5702== For interactive control, run 'callgrind_control -h'.
  [ INFO] [1313713285.708789618]: Initialied ros. node_name: /colorize_clusters_1313713285457983466
  [ INFO] [1313713288.407112127]: Subscribed to topic:/camera/depth_registered/points with queue size of 2

Where the --instr-atstart=no means that valgrind won't actually do
anything.  When my app is running in a steady state, in another window execute::

  % callgrind_control --instr=on
  PID 5702: python ./colorize_clusters.py [requesting '+Instrumentation'...]
  OK.

Now you've started sampling.  Things will slow down... a *lot*.  After
I think callgrind has had time to collect some statistics (say twenty
runs through the graph), provoke a dump::

  % callgrind_control --dump=tenframes_colorized
  PID 5702: python ./colorize_clusters.py [requesting 'Dump tenframes_colorized'...]
  OK.

.. index:: kcachegrind

Now you can run that most awesome of tools `kcachegrind
<http://kcachegrind.sourceforge.net/html/Home.html>`_ on the files that it outputs.

Debugging
---------

Same as above if you're using valgrind's tool ``memcheck``: run
valgrind on the python interpreter, passing the script name as an
argument.  The

