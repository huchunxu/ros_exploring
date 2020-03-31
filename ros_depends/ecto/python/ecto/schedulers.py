#
# Copyright (c) 2014, Yujin Robot
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Yujin Robot nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

###############################################################################
# Description
##############################################################################

"""
.. module:: schedulers
   :platform: Unix
   :synopsis: Utilities for ecto scheduling.


Some extra interfaces that assist with scheduler management.
----

"""

##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import ecto
import threading
try:
    from PySide.QtCore import QTimer
    from PySide.QtGui import QApplication
    HAS_QT = True
except ImportError as e:
    print(e)
    HAS_QT = False

##############################################################################
# Classes
##############################################################################

class MultiPlasmScheduler:
    '''
    Schedules multiple plasms, each one in a separate thread and then
    manages these threads. Useful for combining multiple ecto pipelines.

    If there is a QApplication entity running inside an ecto plasm, this
    ensures that it will get shut down once the threads finish.
    This behaviour can be optionally disabled so shutdown of the main
    QApplication can be handled from elsewhere.

    @param plasm_dict : dictionary of names and plasm pairs to schedule (one plasm per thread)
    @param disable_qt_management : toggle the calls to kill QApplication after the threads finish.
    '''
    def __init__(self, plasm_dict={}, disable_qt_management=False):
        if not HAS_QT:
            disable_qt_management=True
        self.plasms = plasm_dict
        self.schedulers = {}
        self.threads = {}
        for plasm_name in self.plasms:
            self.schedulers[plasm_name] = ecto.Scheduler(self.plasms[plasm_name])
            self.threads[plasm_name] = threading.Thread(name=plasm_name, target=self.schedulers[plasm_name].execute)
        self.disable_qt_management = disable_qt_management
        self.timer = None if disable_qt_management else QTimer()
        self.counter = 0

    def spin(self):
        for thread in self.threads.values():
            thread.start()
        # QApplication.instance().exec_() is blocking, so start a timer and give it a slice
        # of the action every now and then to check if it should shut things down for us.
        if not self.disable_qt_management:
            self.timer.timeout.connect(self.watchdog)
            self.timer.start(200)
            if QApplication.instance() is not None:
                QApplication.instance().exec_()
        for thread in self.threads.values():
            thread.join()

    def watchdog(self):
        for thread in self.threads.values():
            if thread.is_alive():
                return
        if not self.disable_qt_management:
            self.timer.stop()
            QApplication.closeAllWindows()
            QApplication.exit()

    def print_statistics(self):
        for scheduler in self.schedulers.values():
            print(scheduler.stats())
