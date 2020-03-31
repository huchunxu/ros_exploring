/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#include <ecto/cell.hpp>
#include <ecto/registry.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_free.hpp>

namespace boost
{
  namespace serialization
  {
    template<class Archive>
    inline void
    save(Archive & ar, const boost::shared_ptr<ecto::cell> &cell_, const unsigned int file_version)
    {
      std::string type_str = cell_->type();
      ar << type_str;
      std::string instance_name = cell_->name();
      ar << instance_name;
      ar << cell_->parameters << cell_->inputs << cell_->outputs;
    }

    template<class Archive>
    inline void
    load(Archive & ar, boost::shared_ptr<ecto::cell> &cell_, const unsigned int file_version)
    {
      std::string cell_type;
      ar >> cell_type;
      ecto::registry::entry_t e = ecto::registry::lookup(cell_type);
      cell_ = e.construct();
      cell_->declare_params();
      cell_->declare_io();
      std::string instance_name;
      ar >> instance_name;
      cell_->name(instance_name);
      ar >> cell_->parameters >> cell_->inputs >> cell_->outputs;
    }
  }
}

