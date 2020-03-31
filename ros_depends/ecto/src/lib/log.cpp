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
#include <Python.h>
#include <ecto/except.hpp>
#include <ecto/test.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>

using namespace boost;

namespace ecto {

namespace {
  format make_format() {
    // make a format and turn off the 'too many args for format string' error
    // TODO: This is only thread-safe if log() is thread-safe.
    static const char* fmtstr = getenv("ECTO_LOGGING_FORMAT");
    format fmt(fmtstr ? fmtstr : "%14p %25s %40s:%-4u ");
    fmt.exceptions(io::all_error_bits ^ io::too_many_args_bit);
    return fmt;
  }

} // End of anonymous namespace.

bool logging_on() {
  // TODO: This must be made thread-safe independent of log().
  static bool val = getenv("ECTO_LOGGING");
  return val;
}

const static std::string srcdir(SOURCE_DIR);
const static unsigned srcdirlen(srcdir.size()+1);

void log(const char* prettyfn, const char* file, unsigned line, const std::string& msg) {
  // TODO: This must be made thread-safe.
  static format fmt(make_format());
  posix_time::ptime now(posix_time::microsec_clock::local_time());
  const char* file_remainder = file + srcdirlen;
  std::cout << str(fmt % this_thread::get_id() % prettyfn % file_remainder % line)
            << msg << std::endl;
}

void assert_failed(const char* prettyfn, const char* file, unsigned line, const char* cond, const char* msg) {
  log(prettyfn, file, line, str(format("ASSERT FAILED: %s (%s)") % cond % msg));
  abort();
}

} // End of namespace ecto.
