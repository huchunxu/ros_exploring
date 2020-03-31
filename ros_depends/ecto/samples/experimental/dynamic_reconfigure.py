#!/usr/bin/env python
import ecto
from ecto.gui import gui_execute

import ecto_test
import sys


def test_parameter_callbacks():
    generate = ecto_test.Generate()
    param_watcher = ecto_test.ParameterWatcher(value=2)
    sleep = ecto_test.Sleep(seconds=0.2)
    printer = ecto_test.Printer()
    plasm = ecto.Plasm()
    plasm.connect(generate["out"] >> param_watcher["input"],
                  param_watcher['output'] >> printer[:]
        )
    plasm.insert(sleep)
    return gui_execute(plasm)


if __name__ == '__main__':
    sys.exit(test_parameter_callbacks())


