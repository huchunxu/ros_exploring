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
#include <ecto/log.hpp>
#include <ecto/strand.hpp>
#include <ecto/atomic.hpp>
#include <ecto/cell.hpp>
#include <boost/unordered_map.hpp>
#include <boost/asio.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/function.hpp>

namespace ecto {

  struct strand::impl : boost::noncopyable
  {
    boost::scoped_ptr<boost::asio::io_service::strand> asio_strand_p;
  };

  strand::strand() : impl_(new impl)
  {
    ECTO_LOG_DEBUG("Created strand with id %p", id());
  }

  strand::~strand()
  {
  }

  std::size_t strand::id() const
  {
    return reinterpret_cast<std::size_t>(impl_.get());
  }

  void strand::reset()
  {
    ECTO_LOG_DEBUG("Resetting strand %p", impl_->asio_strand_p.get());
    impl_->asio_strand_p.reset();
  }

  bool operator==(const strand& lhs, const strand& rhs)
  {
    return lhs.id() == rhs.id();
  }

  std::size_t strand_hash::operator()(const strand& s) const
  {
    return s.id();
  }

  void on_strand(cell_ptr c, boost::asio::io_service& serv, boost::function<void()> h)
  {
    ECTO_LOG_DEBUG("on_strand %s, serv=%p", c->name() % &serv);
    if (c->strand_) {
      ECTO_LOG_DEBUG("Yup %s should have a strand", c->name());
      //strands_t::scoped_lock l(strands());

      //      const ecto::strand& skey = *(c->strand_);
      //      ECTO_LOG_DEBUG("skey @ %p", &skey);
      //      boost::shared_ptr<boost::asio::io_service::strand>& strand_p = l.value[skey];
      boost::scoped_ptr<boost::asio::io_service::strand>& thestrand = c->strand_->impl_->asio_strand_p;
      if (!thestrand) {
          thestrand.reset(new boost::asio::io_service::strand(serv));
          ECTO_LOG_DEBUG("%s: Allocated new asio::strand @ %p assoc with serv @ %p",
                         c->name() % thestrand.get() % &thestrand->get_io_service());
        }
      else
        {
#if !defined(NDEBUG)

          //home/ecto_ws/recognition_kitchen/ecto/src/lib/strand.cpp:88: error: unused variable ‘serv_inside_strand’
          boost::asio::io_service& serv_inside_strand = thestrand->get_io_service();
          ECTO_LOG_DEBUG("strand matches, %p ??? %p", &serv_inside_strand % &serv);
          ECTO_ASSERT(&serv_inside_strand == &serv,
                      "Hmm, this strand thinks it should be on a different io_service");
#endif
        }
      ECTO_LOG_DEBUG("%s: POST via strand id=%p post to serv %p", c->name() % c->strand_->id() % thestrand.get());
      thestrand->post(h);
    } else {
      ECTO_LOG_DEBUG("%s: POST (strandless) post to serv %p", c->name() % &serv);
      serv.post(h);
    }
  }
}
