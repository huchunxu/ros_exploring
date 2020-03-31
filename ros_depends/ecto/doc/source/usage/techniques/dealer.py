#!/usr/bin/env python

import ecto
import ecto.ecto_test as ecto_test

plasm = ecto.Plasm()
printer = ecto_test.Printer()
cards = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
dealer = ecto.Dealer(tendril=printer.inputs.at('in'), iterable=cards)
plasm.connect(dealer['out'] >> printer['in'])

if __name__ == '__main__':
    Scheduler = ecto.Scheduler
    sched = Scheduler(plasm)
    sched.execute()
    assert dealer.outputs.at('out').type_name == 'double'
    assert dealer.outputs.out == 10
