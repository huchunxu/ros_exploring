#pragma once
#include <ecto/tendril.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/stream_buffer.hpp>

namespace ecto
{
  namespace serialization
  {

    template<typename BufT>
    void
    save(BufT& buf, const tendril& t)
    {
      namespace io = boost::iostreams;
      io::back_insert_device<BufT> inserter(buf);
      io::stream_buffer < io::back_insert_device<BufT> > strbuf(inserter);
      {
        boost::archive::binary_oarchive boa(strbuf, boost::archive::no_header);
        boa << t;
      }
    }

    template<typename BufT>
    void
    load(const BufT& buf, tendril& t)
    {
      namespace io = boost::iostreams;
      io::array_source msgsrc((char*) buf.data(), buf.size());
      io::stream_buffer < io::array_source > msgbufsrc(msgsrc);
      {
        boost::archive::binary_iarchive bia(msgbufsrc, boost::archive::no_header);
        bia >> t;
      }
    }
  }
}
