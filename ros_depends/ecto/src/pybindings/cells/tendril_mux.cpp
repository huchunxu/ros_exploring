/*
 * Copyright 2012 Industrial Perception, Inc.
 * All rights reserved.
 */

#include <ecto/ecto.hpp>

namespace ecto
{
  void deep_copy(tendrils& thiz, const tendrils& that){
      thiz.clear();
      tendrils::const_iterator it = that.begin(), end = that.end();
      while(it != end){
        tendril_ptr t(new tendril(*it->second));//copy tendril value
        thiz.declare(it->first,t);//declare
        ++it;
      }
  }

  struct TendrilMux
  {
    static void
    declare_params(tendrils& parms)
    {
      parms.declare(&TendrilMux::param_tendrils_, "tendrils", "The tendril types to mux.");
    }


    static void
    declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      tendrils_ptr ts;
      parms["tendrils"] >> ts;
      if(!ts) return;
      deep_copy(in, *ts);
      out.declare(&TendrilMux::out_tendrils_, "tendrils", "");
    }

    int process(const tendrils& in, const tendrils& out){
      out_tendrils_->reset(new tendrils);
      deep_copy(**out_tendrils_, in);
      return OK;
    }

    spore<tendrils_ptr> param_tendrils_, out_tendrils_;
  };

  struct TendrilDemux
  {
    static void
    declare_params(tendrils& parms)
    {
      parms.declare<tendrils_ptr>("tendrils", "The tendril types to demux.");
    }

    static void
    declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      tendrils_ptr ts;
      parms["tendrils"] >> ts;
      if(!ts) return;
      deep_copy(out,*ts);
      in.declare(&TendrilDemux::in_tendrils, "tendrils", "");
    }

    int process(const tendrils& in, const tendrils& out){
      tendrils::const_iterator it = (*in_tendrils)->begin(), end = (*in_tendrils)->end();
      while(it != end){
        out[it->first] << it->second;
        ++it;
      }
      return ecto::OK;
    }
    spore<tendrils_ptr> in_tendrils;
  };
}

ECTO_CELL(cells, ecto::TendrilMux, "TendrilMux", "input muxer");
ECTO_CELL(cells, ecto::TendrilDemux, "TendrilDemux", "output demuxer");
