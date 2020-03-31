#!/usr/bin/env python
# 
# Copyright (c) 2012, Industrial Perception, Inc.
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
from ecto import universal_time
from ecto import ptime
from ecto import ptime_duration
from ecto import hours
from ecto import minutes
from ecto import seconds
from ecto import millisec
from ecto import microsec

def check_eq(lhs, rhs):
  assert(lhs == rhs)
  assert(lhs <= rhs)
  assert(lhs >= rhs)
  assert((lhs <  rhs) == False)
  assert((lhs >  rhs) == False)
  assert((lhs != rhs) == False)

# Checks for lhs < rhs
def check_lhs_lt_rhs(lhs, rhs):
  assert(lhs <  rhs)
  assert(lhs <= rhs)
  assert(lhs != rhs)
  assert((lhs >  rhs) == False)
  assert((lhs >= rhs) == False)
  assert((lhs == rhs) == False)

# Checks for lhs > rhs
def check_lhs_gt_rhs(lhs, rhs):
  assert(lhs >  rhs)
  assert(lhs >= rhs)
  assert(lhs != rhs)
  assert((lhs <  rhs) == False)
  assert((lhs <= rhs) == False)
  assert((lhs == rhs) == False)

def test_ptime_bindings():
    t1 = ptime()
    check_eq(t1, t1)

    t2 = t1
    check_eq(t1, t2)
    check_eq(t2, t1)

    t2 = ptime()
    check_eq(t1, t2)
    check_eq(t2, t1)

    t1 = universal_time()
    check_eq(t1, t1)

    t2 = t1
    check_eq(t1, t2)
    check_eq(t2, t1)

    t2 = ptime(t1)
    check_eq(t1, t2)
    check_eq(t2, t1)

    t2 = ptime(str(t1))
    check_eq(t1, t2)
    check_eq(t2, t1)

    t2 = universal_time()
    check_lhs_lt_rhs(t1, t2)
    check_lhs_gt_rhs(t2, t1)

    d0 = ptime_duration()
    check_eq(d0, d0)

    d0 = t1 - t1
    check_eq(d0, d0)

    d1 = t2 - t1
    check_lhs_lt_rhs(d0, d1)
    check_lhs_gt_rhs(d1, d0)

    # d2 is negative: Should have d2 < d0 < d1
    d2 = t1 - t2
    check_lhs_lt_rhs(d2, d0)
    check_lhs_gt_rhs(d0, d2)
    check_lhs_lt_rhs(d2, d1)
    check_lhs_gt_rhs(d1, d2)
    check_lhs_lt_rhs(d0, d1)
    check_lhs_gt_rhs(d1, d0)

    d3 = d1 + d2
    check_eq(d0, d3)

    t3 = t1 + d1
    check_eq(t2, t3)

    h1 = hours(1)
    check_eq(h1, h1)
    h2 = hours(2)
    check_lhs_lt_rhs(h1, h2)
    check_lhs_gt_rhs(h2, h1)

    m1 = minutes(1)
    check_eq(m1, m1)
    check_lhs_lt_rhs(m1, h1)
    check_lhs_gt_rhs(h1, m1)
    m2 = minutes(2)
    check_lhs_lt_rhs(m1, m2)
    check_lhs_gt_rhs(m2, m1)

    s1 = seconds(1)
    check_eq(s1, s1)
    check_lhs_lt_rhs(s1, m1)
    check_lhs_gt_rhs(m1, s1)
    s2 = seconds(2)
    check_lhs_lt_rhs(s1, s2)
    check_lhs_gt_rhs(s2, s1)

    ms1 = millisec(1)
    check_eq(ms1, ms1)
    check_lhs_lt_rhs(ms1, s1)
    check_lhs_gt_rhs(s1, ms1)
    ms2 = millisec(2)
    check_lhs_lt_rhs(ms1, ms2)
    check_lhs_gt_rhs(ms2, ms1)

    us1 = microsec(1)
    check_eq(us1, us1)
    check_lhs_lt_rhs(us1, ms1)
    check_lhs_gt_rhs(ms1, us1)
    us2 = microsec(2)
    check_lhs_lt_rhs(us1, us2)
    check_lhs_gt_rhs(us2, us1)

if __name__ == '__main__':
    test_ptime_bindings()
