#!/usr/bin/env python
import ecto

ts = ecto.Tendrils()
ts.declare('foo', ecto.Tendril.createT('std::string'))

ts_pass = ecto.PassthroughTendrils(tendrils=ts)

ts.foo = "hello"
ts_pass.process()
assert ts.foo == "hello"
assert ts_pass.outputs.foo == "hello"
assert ts_pass.inputs.foo == "hello"

ts_pass.inputs.foo = "world"
assert ts.foo == "world"
assert ts_pass.outputs.foo == "world"
assert ts_pass.inputs.foo == "world"

