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
ectomodule(ecto_test
  Accumulator.cpp
  Add.cpp
  BreakEveryN.cpp
  BpObjectToCellPtr.cpp
  CantCallMeFromTwoThreads.cpp
  ConfigureCalledOnce.cpp
  DontCallMeFromTwoThreads.cpp
  DoOverFor.cpp
  EmitAndAccept.cpp
  ExceptInConstructor.cpp
  FileIO.cpp
  Multiply.cpp
  Quitter.cpp
  QuitAfter.cpp
  Increment.cpp
  LatticeSleep.cpp
  Printer.cpp
  Ping.cpp
  Metrics.cpp
  Gather.cpp
  Generate.cpp
  RequiredInput.cpp
  RequiredParam.cpp
  RequiredIO.cpp
  Passthrough.cpp
  ParameterWatcher.cpp
  Sleep.cpp
  SleepPyObjectAbuser.cpp
  StartStopCounter.cpp
  Throttle.cpp
  ThrowAfter.cpp
  Uniform01.cpp
  throws_in_handler.cpp
  gil_exercise.cpp
  ecto_test.cpp
  DESTINATION  ecto
  INSTALL
)
