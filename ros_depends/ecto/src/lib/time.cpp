#include <ecto/time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
namespace ecto
{
  namespace pt = boost::posix_time;
  stamp_t microseconds(){
     static pt::ptime start_time = pt::microsec_clock::universal_time();
     pt::time_duration delta = pt::microsec_clock::universal_time() - start_time;
     return delta.total_microseconds();
  }

  ptime_t local_time(){
    return pt::microsec_clock::local_time();
  }

  ptime_t universal_time(){
    return pt::microsec_clock::universal_time();
  }
}
