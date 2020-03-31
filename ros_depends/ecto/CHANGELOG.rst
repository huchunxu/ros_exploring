0.6.12 (2016-04-17)
-------------------
* fix tests on Kinetic
* fix PySide dependency
* add missing implementation for executing
  That fixes `#233 <https://github.com/plasmodic/ecto/issues/233>`_
* install the ecto library of test cells for users to utilise in their own tests.
* checking for ecto-test target existence
* Contributors: Daniel Stonier, Vincent Rabaud, edgarriba

0.6.11 (2015-09-21)
-------------------
* minor doc syntax fixes
* saner installation instructions from source.
* add missing includes to make headers compile standalone
  - add cstddef to introduce std::size_t
  - add parameters.hpp to its implementation (works because of #pragma once)
* fix bsd-license check script
  Not all Copyright is (1.) from 2011 and (2.) by Willow Garage
  autofix is undefined if ECTO_LICENSE_AUTOFIX is not exported.
* Re-add thread library to linker list
  Regression caused by 8e354b5aa8281ea8117fc93adb290998b7810be7
* docs about various entities that affect graph execution, also other minor cleanups.
  Removed the redundancies in the install docs (2x install instructions and 2x dependencies)
  and cleaned the place up a bit.
* Provide test for ecto::BREAK return value.
* implement ecto::BREAK behavior
  This patch makes ecto schedule the next iteration
  through the plasm with ecto::BREAK as discussed in
  https://github.com/plasmodic/ecto/issues/251
* remove some old Willow Garage URLs
* update doc's url
* Contributors: Daniel Stonier, Michael Görner, Po-Jen Lai, Scott K Logan, Vincent Rabaud, v4hn

0.6.10 (2015-07-22)
-------------------
* update test for future fix
* potentially fix compilation with Boost 1.58
  could be a fix for `#275 <https://github.com/plasmodic/ecto/issues/275>`_
* Contributors: Vincent Rabaud

0.6.9 (2015-05-08)
------------------
* re-add some Python tests
* add missing PySide dependency
* Contributors: Vincent Rabaud

0.6.8 (2015-03-13)
------------------
* threadable plasms update.
  This reintroduces some level of threading into ecto. It was initially
  removed in c2a24a51, but this left some legacy code
  wasn't working as expressed in the documentation. These updates
  allow parallel execution of plasms in separate threads. It doesn't
  quite reach the scope of the original plan for ecto (threading for
  free *inside* a plasms computation), but it does cover several useful
  scenarios.
* convenience class for scheduling plasms across threads.
* Reformatted cell processing result handler (trivial).
  For clarity, and points back to an issue created which discusses
  the BREAK/CONTINUE options.
* Don't clobber the underlying c++ executions when aborting, be graceful.
  Previously used PyErr_SetInterrupt which would send a keyboard interrupt
  back to the controlling python script. However we don't want to abort
  in this way - it is preferable to let the execution gracefully bow out.
  More information and the actual code that does the bowing out is in
  pull request `#250 <https://github.com/plasmodic/ecto/issues/250>`_.
* activate gil release/call macros
* Be free of the GIL!
  This lets ecto pipeline processing be free of its evil overlord, the [GIL](https://wiki.python.org/moin/GlobalInterpreterLock). That is, you can now schedule different plasms in different threads...e.g. some pseudocode:
  # ...construct some plasms
  image_scheduler = ecto.Scheduler(image_plasm)
  odometry_scheduler = ecto.Scheduler(odometry_plasm)
  image_thread = threading.Thread(name="image_thread", target=image_scheduler.execute)
  odometry_thread = threading.Thread(name="odometry_thread", target=odometry_scheduler.execute)
  image_thread.start()
  odometry_thread.start()
  image_thread.join()
  odometry_thread.join()
  ```
  Of course, inside a cell's `process()` call, you can always grab the GIL momentarily if you need to using the opposite macro...`ECTO_SCOPED_CALLPYTHON`.
* Bugfix python lookup for internal class variables
  Probably went unnoticed for a long time since it wasn't used. Came across this (I think - was quite a while ago) while fleshing out pythonic cell construction with names and parameter args. Should have gone in at the same time as 39b9cad9.
* fix doc
  fixes `#239 <https://github.com/plasmodic/ecto/issues/239>`_
* Don't clobber the underlying c++ executions when aborting, be graceful.
  Previously used PyErr_SetInterrupt which would send a keyboard interrupt
  back to the controlling python script. However we don't want to abort
  in this way - it is preferable to let the execution gracefully bow out.
  More information and the actual code that does the bowing out is in
  pull request `#250 <https://github.com/plasmodic/ecto/issues/250>`_.
* activate gil release/call macros
* Bugfix python lookup for internal class variables
  Probably went unnoticed for a long time since it wasn't used. Came across this (I think - was quite a while ago) while fleshing out pythonic cell construction with names and parameter args. Should have gone in at the same time as 39b9cad9.
* fix doc
  fixes `#239 <https://github.com/plasmodic/ecto/issues/239>`_
* tiers need a boost prefix with boost 1.57
* Contributors: Daniel Stonier, Michael Görner, Vincent Rabaud

0.6.7 (2014-11-11)
------------------
* Merge branch 'master' of github.com:plasmodic/ecto
* Merge pull request `#266 <https://github.com/plasmodic/ecto/issues/266>`_ from stonier/if_nests
  Nested If Cells
* test for nested ifs.
* parameterise if cell name variables, this allows if-nests.
* add awesome Daniel as maintainer
* Merge pull request `#265 <https://github.com/plasmodic/ecto/issues/265>`_ from stonier/gil_releaser
  Gil releaser
* Merge pull request `#264 <https://github.com/plasmodic/ecto/issues/264>`_ from stonier/directed_configuration
  Directed configuration revisited
* bugfix module reference in the gil exercise.
* advanced multithreaded gil releaser.
* directed configuration
* Contributors: Daniel Stonier, Vincent Rabaud

0.6.6 (2014-09-06)
------------------
* don't forget to install necessary extra CMake files
* Contributors: Vincent Rabaud

0.6.5 (2014-09-06)
------------------
* do not install dev ectoConfig*
  fixes `#262 <https://github.com/plasmodic/ecto/issues/262>`_
* Merge pull request `#261 <https://github.com/plasmodic/ecto/issues/261>`_ from stonier/bugfix_pythonic_cells
  Bugfix pythonic cell construction
* Merge pull request `#263 <https://github.com/plasmodic/ecto/issues/263>`_ from stonier/dot_graph_titles
  Add title to the dot graph windows
* add title to the dot graph windows.
* bugfix pythonic cell construction and parameter argument invocation.
* Merge pull request `#259 <https://github.com/plasmodic/ecto/issues/259>`_ from stonier/dynamic_reconfigure_whitelist
  A whitelist for dynamic reconfigure
* Merge pull request `#254 <https://github.com/plasmodic/ecto/issues/254>`_ from stonier/optional_inputs
  optional processing of connected inputs only
* Merge pull request `#258 <https://github.com/plasmodic/ecto/issues/258>`_ from stonier/dynamic_reconfigure
  Boolean types for dynamic reconfigure
* Use the value, don't assume it is true.
* a whitelist for dynamic reconfigure.
* bugfix boolean parameter setting in dynamic reconfigure.
* checkboxes for dynamic reconfigure.
* optional processing of connected inputs only.
* Contributors: Daniel Stonier, Vincent Rabaud

0.6.4 (2014-07-27)
------------------
* Posix signal handling
* proper depends syntax for pythonlibs find-packages variables.
* add the impl sub-package to the python setup.
* subprocessing view_plasm
* sigint handling for sync and async executors
* remove usage of SYSTEM in include_directories.
* experimental signal connection for sync executions.
* internal 0.4 xdot is broken on trusty, this moves it out to 0.5 as a rosdep entity.
* Contributors: Daniel Stonier, Vincent Rabaud

0.6.3 (2014-04-03)
------------------
* Merge pull request `#247 <https://github.com/plasmodic/ecto/issues/247>`_ from cottsay/master
  Added depend on python
* Added depend on python
* Contributors: Scott K Logan, Vincent Rabaud

0.6.2 (2014-03-02)
------------------
* get the tests to compile on OSX
* solve boost::bind problem on some compilers
* fixes `#245 <https://github.com/plasmodic/ecto/issues/245>`_ according to http://bugs.python.org/issue10910
* trust catkin to handle the version number
* trust catkin to handle ecto_LIBRARIES
* Contributors: Vincent Rabaud

0.6.1 (2014-02-16)
------------------
* get tests to pass with boost 1.54
* update maintainers
* fix compilation on Saucy
* fix warnings in the doc
* Contributors: Vincent Rabaud

0.6.0 (2014-01-26  15:37:06 +0100)
----------------------------------
- drop Fuerte support
- fix compilation errors on recent boost
