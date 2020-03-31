Ecto - A C++/Python Computation Graph Framework
===============================================

Initially aimed at computer vision and perception research tasks, Ecto is a hybrid C++/Python framework for organizing computations as directed acyclic graphs of computing 'cells' connected by typed edges. These graphs are typically constructed via Python script and executed in a single process (and possibly multiple threads) by external schedulers. The computing nodes are written in C++ against a simple interface that naturally creates libraries of self-documenting, scriptable components and smooths the path from prototyping to testing to deployment.

Ecto itself is small, has minimal dependencies (C++, Boost, Python) and works with or without OpenCV, PCL, and ROS in any combination. We believe that Ecto allows vision and perception researchers to express their computational models in a natural fashion, obviating e.g. ROS time synchronizers and ROS nodelets in most cases.

As of 2012, Ecto is officially released. It is being used by researchers at Willow Garage and in industry for prototype applications of object capture and modelling, `object recognition <http://wg-perception.github.com/object_recognition_core/>`_, pose estimation and refinement, visual slam, projector-based augmented reality and chess playing.


To understand ecto, you probably want to go over the Ecto docs first:

.. toctree::
   :maxdepth: 1

   Overview <ecto/overview/index.rst>
   Install <ecto/install/index.rst>
   Usage <ecto/usage/index.rst>
   ecto/reference/index.rst

.. rubric:: Ecto modules

Ecto has several modules that wrap different libraries or functionalities:

   * :ref:`ecto_image_pipeline <ectoimagepipeline:ecto_image_pipeline>`
   * :ref:`ecto_opencv <ectoopencv:ecto_opencv>`
   * :ref:`ecto_openni <ectoopenni:ecto_openni>`
   * :ref:`ecto_pcl <ectopcl:ecto_pcl>`
   * :ref:`ecto_ros <ectoros:ecto_ros>`

.. rubric:: Bug reports

Please use the github infrastructure fot the desired module on https://github.com/plasmodic

.. rubric:: Email List

Also feel free to join the email list:

* site: http://groups.google.com/a/plasmodic.org/group/ecto-dev
