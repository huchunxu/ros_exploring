#!/usr/bin/env python
import ecto, math

def make_tendrils():
    ts = ecto.Tendrils()
    ts.declare('foo', ecto.make_tendril('std::string'))
    ts.declare('bar', ecto.make_tendril('double'))
    return ts

ts_mux = ecto.TendrilMux(tendrils=make_tendrils())
ts_demux = ecto.TendrilDemux(tendrils=make_tendrils())

ts_mux.inputs.foo = "hello"
ts_mux.inputs.bar = math.pi

plasm = ecto.Plasm()
plasm.connect(ts_mux['tendrils'] >> ts_demux['tendrils'])

sched = ecto.Scheduler(plasm)
sched.execute(niter=1)

assert ts_mux.outputs.tendrils.foo == ts_mux.inputs.foo
assert dict(ts_mux.outputs.tendrils) == dict(ts_demux.outputs)
