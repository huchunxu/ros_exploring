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
#include <ecto/except.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>

using namespace boost;

#define ECTO_STRESS_TEST
#ifdef ECTO_STRESS_TEST
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/lexical_cast.hpp>

namespace ecto {
  namespace test {

    template <typename T>
    T get_from_env_with_default(const char* name, T defval)
    {
      const char* envval = getenv(name);
      if (!envval)
        return defval;

      T casted = boost::lexical_cast<T>(envval);
      return casted;
    }

    template unsigned get_from_env_with_default(const char*, unsigned);
    template int get_from_env_with_default(const char*, int);

    const unsigned max_delay = get_from_env_with_default("ECTO_MAX_DELAY", 0); // 10 mst
    const unsigned min_delay = get_from_env_with_default("ECTO_MIN_DELAY", 10); // 10 mst
    const unsigned delay_seed = get_from_env_with_default("ECTO_DELAY_SEED", time(0));

    struct tls {

      boost::mt19937 gen;
      boost::uniform_int<unsigned> dist;

      tls() : gen(delay_seed), dist(0, max_delay) { }

      inline void rndsleep() {
        unsigned dur = dist(gen);
        if (max_delay > 0 && dur >= min_delay)
          usleep(dur);
      }
    };

    void random_delay()
    {
      static boost::thread_specific_ptr<ecto::test::tls> bp;
      if (__builtin_expect(!bp.get(), 0))
        bp.reset(new ecto::test::tls);
      bp->rndsleep();
    }

  }
}

#endif
