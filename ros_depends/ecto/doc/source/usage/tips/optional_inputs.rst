.. _optional-inputs:

Optional Inputs
===============

By default, input tendrils to a cell are assumed to be always connected and active
(with at least some default value). There are times however when you may wish to
introspect the connected inputs and react according to what input tendrils are active.
This can provide you with increased flexibility at a small cost (i.e. you don't need
to build an entirely new cell) or more esoteric situations where you
drop your cell into multiple pipelines that are possibly stretched across threads - in
these its behaviour adapts to the inputs.

Simple Example
--------------

By default, optional tendril introspection is disabled. To turn it on for your cell,
simply enable the ``connected_inputs_only`` parameter for the cell.

.. code-block:: python

   accumulator = ecto_test.Accumulator("Accumulator", connected_inputs_only=True)

To introspect from within your cell's ``process()`` call, simply look up the map to
determine if your variable exists. For example, the above ``Accumulator`` cell might look like:

.. code-block:: cpp

   struct Accumulator
   {
     static void declare_io(const tendrils& p, tendrils& i, tendrils& o)
     {
       i.declare<double>(&Accumulator::left_,"left", "Left hand operand.");
       i.declare<double>(&Accumulator::right_,"right","Right hand operand.");
       o.declare<double>(&Accumulator::out_,"out","The current accumulation.", 0.0);
     }
     int process(const tendrils& inputs, const tendrils& /*outputs*/)
     {
       if ( inputs.find("left") != inputs.end() ) {
         *out_ += *left_;
       }
       if ( inputs.find("right") != inputs.end() ) {
         *out_ += *right_;
       }
       return ecto::OK;
     }
     spore<double> out_, left_, right_;
   };





