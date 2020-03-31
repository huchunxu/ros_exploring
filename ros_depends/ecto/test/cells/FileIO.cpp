#include <ecto/ecto.hpp>
#include <iostream>
using ecto::tendrils;
namespace ecto_test
{
  typedef boost::shared_ptr<std::ostream> ostream_ptr;
  typedef boost::shared_ptr<std::istream> istream_ptr;

  struct FileO
  {
    static void
    declare_params(ecto::tendrils& parameters)
    {
      parameters.declare<ostream_ptr>("file", "A filelike object");
    }
    static void
    declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<double>("input", "A double input.");
    }
    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      stream_ = p["file"];
      input_ = i["input"];
    }
    int
    process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
    {
      ECTO_SCOPED_CALLPYTHON();
      **stream_ << *input_ << std::endl;
      return ecto::OK;
    }
    ecto::spore<double> input_;
    ecto::spore<ostream_ptr> stream_;
  };

  struct FileI
  {
    static void
    declare_params(ecto::tendrils& parameters)
    {
      parameters.declare<istream_ptr>("file", "A filelike object");
    }
    static void
    declare_io(const ecto::tendrils& parameters,
               ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      outputs.declare<double>("output", "A double output.");
    }
    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      stream_ = p["file"];
      output_ = o["output"];
    }
    int
    process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
    {
      ECTO_SCOPED_CALLPYTHON();
      std::istream& stream = **stream_;
      if (stream.eof()) return ecto::QUIT;
      double d;
      stream >> d;
      if (!stream.good()) return ecto::QUIT;
      *output_ = d;
      return ecto::OK;
    }
    ecto::spore<double> output_;
    ecto::spore<istream_ptr> stream_;
  };
}

ECTO_CELL(ecto_test, ecto_test::FileO, "FileO", "Writes doubles to a file like object");
ECTO_CELL(ecto_test, ecto_test::FileI, "FileI", "Reads doubles from a file like object");
