#include <ecto/ecto.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using ecto::tendrils;
using ecto::spore;

namespace ecto_test
{

  struct Accumulator
  {
    static void declare_io(const tendrils& p, tendrils& i, tendrils& o)
    {
      i.declare<double>(&Accumulator::left_,"left", "Left hand operand.");
      i.declare<double>(&Accumulator::right_,"right","Right hand operand.");
      o.declare<double>(&Accumulator::out_,"out","The current accumulation.", 0.0);
    }
    int process(const tendrils& inputs, const tendrils& /*outputs*/)
    {
      boost::mutex::scoped_lock lock(mutex);
      if ( inputs.find("left") != inputs.end() ) {
        std::cout << "  Left: " << *out_ << "+" << *left_ << "=" << *out_ + *left_ << std::endl;
        *out_ += *left_;
      }
      if ( inputs.find("right") != inputs.end() ) {
        std::cout << "  Right: " << *out_ << "+" << *right_ << "=" << *out_ + *right_ << std::endl;
        *out_ += *right_;
      }
      // sleep a bit to effectively test the sharing
      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
      return ecto::OK;
    }
    spore<double> out_, left_, right_;
    mutable boost::mutex mutex;
  };
}

ECTO_CELL(ecto_test, ecto_test::Accumulator, "Accumulator", "Add inputs (potentially from different threads) to an incrementally accumulating sum.");
