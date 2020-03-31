/*
 * Copyright 2012 Industrial Perception, Inc.
 * All rights reserved.
 */

#include <ecto/ecto.hpp>

namespace ecto
{
  struct PassthroughTendrils
  {
    static void
    declare_params(tendrils& parms)
    {
      parms.declare<tendrils_ptr>("tendrils", "The tendrils to pass through");
    }

    static void
    declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      tendrils_ptr ts;
      parms["tendrils"] >> ts;
      if(!ts)return;
      tendrils::const_iterator it = ts->begin(), end = ts->end();
      while(it != end){
        in.declare(it->first, it->second);
        out.declare(it->first, it->second);
        ++it;
      }
    }
  };
}

ECTO_CELL(cells, ecto::PassthroughTendrils, "PassthroughTendrils", "Passes through a tendrils object."
          " All inputs and outputs will point at the same tendril pointers.")
