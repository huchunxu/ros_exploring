#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
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
#
import ecto
import ecto.ecto_test as ecto_test
import sys
from ecto.test import test

def build_pipelines():

    left_plasm = ecto.Plasm()
    right_plasm = ecto.Plasm()
    accumulator = ecto_test.Accumulator("Accumulator", connected_inputs_only=True)
    left_generator = ecto_test.Generate("Left Generator", step=1.0, start=1.0, stop=10.0)
    right_generator = ecto_test.Generate("Right Generator", step=2.0, start=1.0, stop=10.0)
    left_plasm.connect(left_generator["out"] >> accumulator["left"])
    right_plasm.connect(right_generator["out"] >> accumulator["right"])
    plasms = {
          "left": left_plasm,
          "right": right_plasm
         }
    scheduler = ecto.MultiPlasmScheduler(plasms, disable_qt_management=True)
    return (accumulator, scheduler)


@test
def test_plasm():
    (accumulator, scheduler) = build_pipelines()
    scheduler.spin()
    print( "RESULT: %s" % accumulator.outputs.out)
    assert accumulator.outputs.out == 80.0

if __name__ == '__main__':
    test_plasm()
