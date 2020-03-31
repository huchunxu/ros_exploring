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

#include <ecto/config.hpp>

#if defined(ECTO_LOGGING)
#include <boost/format.hpp>
#endif
#include <ecto/util.hpp>
#include <ecto/test.hpp>
#include <stdint.h>

namespace ecto {
  ECTO_EXPORT void log(const char*, const char*, unsigned line, const std::string& msg);
  ECTO_EXPORT void assert_failed(const char*, const char* file, unsigned line, const char* cond, const char* msg);
  bool logging_on();
}

#ifdef NDEBUG
#define ECTO_ASSERT(X, msg) do { ECTO_RANDOM_DELAY(); } while(false)
#else
#define ECTO_ASSERT(X, msg)                                             \
  do {                                                                  \
    ECTO_RANDOM_DELAY();                                                \
    if (X) ; else ecto::assert_failed(__PRETTY_FUNCTION__, __FILE__, __LINE__, #X, msg); \
  } while(false)
#endif

#if defined(ECTO_LOGGING)
#define ECTO_LOG_DEBUG(fmt, args)                                       \
  do {                                                                  \
    ECTO_RANDOM_DELAY();                                                \
    if (__builtin_expect((ecto::logging_on()), 0))                      \
      ::ecto::log(__PRETTY_FUNCTION__, __FILE__, __LINE__, str(boost::format(fmt) % args)); \
  } while (false)
#define ECTO_START()  ECTO_LOG_DEBUG(">>> %s", __PRETTY_FUNCTION__);
#define ECTO_FINISH() ECTO_LOG_DEBUG("<<< %s", __PRETTY_FUNCTION__);
#else
#define ECTO_LOG_DEBUG(fmg, args) do { ECTO_RANDOM_DELAY(); } while (false)
#define ECTO_START() do { ECTO_RANDOM_DELAY(); } while(false)
#define ECTO_FINISH() do { ECTO_RANDOM_DELAY(); } while(false)
#endif

#ifdef ECTO_TRACE_EXCEPTIONS
#define ECTO_TRACE_EXCEPTION(E) ECTO_LOG_DEBUG("CAUGHT: %s", E);
#else
#define ECTO_TRACE_EXCEPTION(E)
#endif

