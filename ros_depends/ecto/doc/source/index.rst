:orphan:

Ecto
====

Ecto is a hybrid C++/Python development framework for constructing
efficient processing pipelines.  In Ecto, pipelines are defined in
terms of processing units, ``Cells``, connected by data paths,
``Tendrils``, that form *Directed Acyclic Graphs*, ``Plasms``. Cells
are typically written in C++, tendrils may be any copyable type, and
the flow of data through plasms is controlled by various external
schedulers.  Graphs are constructed with a small domain-specific
language hosted in Python.

Ecto may be useful in problem domains such as perception, audio, or
robotics.


.. rubric:: Table of Contents

.. toctree::
   :maxdepth: 1

   overview/index.rst
   Install <install/index.rst>
   Usage <usage/index.rst>
   reference/index.rst  


.. rubric:: Bug reports

Please use the github infrastructure https://github.com/plasmodic/ecto

.. rubric:: Email List

Also feel free to join the email list:

* site: http://groups.google.com/a/plasmodic.org/group/ecto-dev

.. rubric:: Ecto Developer

If you develop ecto, you want to read the following:

.. toctree::
   :maxdepth: 1

   motivation.rst
   codingstandards.rst
   changelog.rst
   release_checklist.rst
