// 
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// 
#include <ecto/ecto.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/python/overloads.hpp>
namespace bp = boost::python;
namespace ecto
{
  struct Dealer
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<bp::object>("iterable", 
                            "iterable python object... values to be output")
        .required(true);
      p.declare<tendril_ptr>("tendril",
                              "Destination tendril...  used to set output type")
        .required(true);
    }

    static void
    declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      out.declare<tendril::none>("out", "Any type");
    }

    void
    configure(const tendrils& p, const tendrils& in, const tendrils& out)
    {
      ECTO_SCOPED_CALLPYTHON();
      bp::object iterable = p["iterable"]->get<bp::object>();

      size_t end = bp::len(iterable);

      tendril_ptr typer = p["tendril"]->get<tendril_ptr>();

      for (size_t j = 0; j < end; ++j)
        {
          bp::object value = iterable[j];
          tendril x;
          x << *typer; // set the type.
          x << value;  // extract from python
          values_.push_back(x);
        }

      out_ = out["out"];
      // initialise so directed configuration has something to play with
      if (!values_.empty()) {
        *out_ << values_.front();
      }
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      if (values_.empty())
        return ecto::QUIT;
      *out_ << values_.front();
      values_.pop_front();
      return ecto::OK;
    }
    std::list<tendril> values_;
    tendril_ptr out_;
  };
}
ECTO_CELL(cells, ecto::Dealer, "Dealer", "Emit values of python iterable");

