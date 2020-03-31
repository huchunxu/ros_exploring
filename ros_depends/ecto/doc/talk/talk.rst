.. include:: <s5defs.txt>

.. image:: ecto_3x3.svg
   :class: logo
   :width: 75 %
   :align: center

.. container::
   :align: center
   :class: logo

   parallel framework for perception


.. container::
   :align: center
   :class: logo

   troy straszheim and ethan rublee


the design space
================

* interprocess vs intraprocess
* single- vs multi-threaded
* synchronous vs asynchronous
* compiled vs interpreted

LOC
===

Amoeba (Beta, August 2011)

Development began in earnest August 2010.

* Ecto core:

  * 7k lines C++
  * 2.5k lines Python 

* Tests:
  * 1.8k lines C++
  * 3.3k lines Python

> 2k lines of handwritten .rst documentation

Across projects ``ecto``, ``ecto_pcl``, ``ecto_ros``, ``ecto_opencv``,
28k lines C++, 9.5k lines Python



dig it
======

Multithreaded... or singlethreaded.  You choose.  At runtime.  Without
recompiling.




* an attack on the manycore problem

* semantics of copying

* task parallelism vs data parallelism: allow individual cells to be
  as data-parallel as one likes.  Inherently task-parallel, where each
  cell performs a task: individual ticks of data execute concurrently,
  and individual tasks within the processing of that tick of data
  execute concurrently.

"The biggest sea change in software development since the OO
revolution is knocking at the door, and its name is Concurrency."
- Herb Sutter, "The Free Lunch Is Over", 2005

goals
=====

give researchers a mental model that makes them more productive and
naturally expresses task-parallelism

support but don't force the use of primarily data-parallel
technologies like CUDA... even if every operation happens on a
massively parallel coprocessor (like a gpu) you *still* need a way to
organize them.

scale to many cores w/o recompiling

make individual algorithms and *components* of algorithms easily
reusable

be more general than vision/perception/robotics... audio?  general machine learning?

portability:  unix/osx/windows

buildable and usable with standard tools... easy for experienced developers to grok

as introspectiable and self-documenting as possible

minimal system dependencies:  boost, python, cmake.

optionality of python

no environment variables beyond (PYTHON|LD_LIBRARY)_PATH

easy integration with other systems

embedding in other 


target users
------------

* researchers (vision|perception):  code in C++, run via python
* tinkerers, end-users:  script in python


future work
===========

new schedulers
processor affinity;  topology of graph on hardware
reverse edges
integration





