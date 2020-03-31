#pragma once
#include <stdint.h>//int64_t
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>

namespace ecto{
  typedef uint64_t stamp_t;
  typedef boost::posix_time::ptime ptime_t;
  typedef boost::posix_time::time_duration ptime_duration_t;
  typedef boost::posix_time::hours hours_t;
  typedef boost::posix_time::minutes minutes_t;
  typedef boost::posix_time::seconds seconds_t;
  typedef boost::posix_time::millisec millisec_t;
  typedef boost::posix_time::microsec microsec_t;
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
  typedef boost::posix_time::nanosec nanosec_t;
#endif

  /**
   * The number of microseconds since the first call to this function.
   * @return microseconds, i.e. 10-6 seconds
   */
  stamp_t microseconds();
  ptime_t local_time();
  ptime_t universal_time();
}
