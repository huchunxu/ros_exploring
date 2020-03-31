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

#include <boost/version.hpp>
#if BOOST_VERSION <= 104000
#pragma GCC diagnostic ignored "-Wsign-compare"
// TODO EAR meh noisy -- We should make a portable stable archive type.
//#pragma message "Ignoring signed-unsigned comparison in boost::serialization in 1.40"
#endif

#include <ecto/ecto.hpp>
#include <ecto/plasm.hpp>
#include <ecto/edge.hpp>
#include <ecto/registry.hpp>
#include <ecto/vertex.hpp>

#include <ecto/graph/types.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/date_time/gregorian/greg_serialize.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>

#include <ecto/serialization/cell.hpp>
#include <ecto/serialization/registry.hpp>
#include <ecto/serialization/cell.hpp>

#include <boost/tuple/tuple.hpp>

ECTO_REGISTER_SERIALIZERS(std::string);

ECTO_REGISTER_SERIALIZERS(float);
ECTO_REGISTER_SERIALIZERS(double);

ECTO_REGISTER_SERIALIZERS(char);
ECTO_REGISTER_SERIALIZERS(short);
ECTO_REGISTER_SERIALIZERS(int);
ECTO_REGISTER_SERIALIZERS(long);

ECTO_REGISTER_SERIALIZERS(unsigned char);
ECTO_REGISTER_SERIALIZERS(unsigned short);
ECTO_REGISTER_SERIALIZERS(unsigned int);
ECTO_REGISTER_SERIALIZERS(unsigned long);

ECTO_REGISTER_SERIALIZERS(bool);

ECTO_REGISTER_SERIALIZERS(std::vector<int>);
ECTO_REGISTER_SERIALIZERS(std::vector<float>);
ECTO_REGISTER_SERIALIZERS(std::vector<double>);

ECTO_REGISTER_SERIALIZERS(boost::posix_time::ptime);

namespace boost
{
  namespace serialization
  {
    template<class Archive>
    void
    serialize(Archive &, ecto::tendril::none, const unsigned int)
    { }
  } // namespace serialization
} // namespace boost

ECTO_REGISTER_SERIALIZERS(ecto::tendril::none);

ECTO_REGISTER_SERIALIZERS(ecto::cell::ptr);
ECTO_REGISTER_SERIALIZERS(ecto::tendrils_ptr);
ECTO_REGISTER_SERIALIZERS(ecto::tendril_ptr);

namespace ecto
{
  template<class Archive>
  void
  tendril::serialize(Archive& ar, const unsigned int)
  {
    ECTO_LOG_DEBUG("%s", "serializing");
    std::string typename_;
    if (typename Archive::is_saving())
      typename_ = type_ID_;

    ar & typename_;
    ECTO_LOG_DEBUG("typename = %s", typename_);
    ar & doc_;

    ecto::serialization::registry<Archive>::instance()
      .serialize(typename_, ar, const_cast< ::ecto::tendril&>(*this));
  }

  ECTO_INSTANTIATE_SERIALIZATION(tendril);

  template<class Archive>
  void
  tendrils::serialize(Archive & ar, const unsigned int version)
  {
    ECTO_LOG_DEBUG("%s", "serializing");
    ar & storage;
  }
  ECTO_INSTANTIATE_SERIALIZATION(tendrils);

  namespace serialization
  {
    template<typename Archive>
    void
    registry<Archive>::serialize(const std::string& key, Archive& ar, tendril& t) const
    {
      ECTO_LOG_DEBUG("%s", "serializing");
      typename serial_map_t::const_iterator it = serial_map.find(key);
      if (it == serial_map.end())
      {
        throw std::logic_error("Could not find a serializer registered for the type: " + key);
      }
      it->second(ar, t);
    }

    template<typename Archive>
    void
    registry<Archive>::add(const std::string& name, serial_fn_t fnc)
    {
      typename serial_map_t::iterator it;
      bool inserted;
      ::boost::tie(it, inserted) = serial_map.insert(std::make_pair(name, fnc));
      if (!inserted)
      {
        std::cerr << "Warning: ignoring non novel serialization for " << name << "." << std::endl;
      }
    }

    template<typename Archive>
    registry<Archive>&
    registry<Archive>::instance()
    {
      static registry<Archive> instance_;
      return instance_;
    }

    template<typename Archive>
    registry<Archive>::registry()
    {
    }

    template struct registry< ::boost::archive::binary_oarchive> ;
    template struct registry< ::boost::archive::binary_iarchive> ;
  }
}


namespace ecto
{
  namespace serialization
  {
    typedef ecto::graph::graph_t::vertex_descriptor vd_t;
    typedef boost::tuple<vd_t, std::string, vd_t, std::string> connection_t;
    typedef std::map<vd_t, cell::ptr> cell_map_t;
  }

  template<class Archive>
  void
  plasm::save(Archive & ar, const unsigned int version) const
  {
    const ecto::graph::graph_t& g = graph();
    ecto::graph::graph_t::edge_iterator begin, end;
    serialization::vd_t source, sink;
    serialization::cell_map_t cell_map;
    std::vector<serialization::connection_t> connections;
    for (boost::tie(begin, end) = boost::edges(g); begin != end; ++begin)
    {
      source = boost::source(*begin, g);
      sink = boost::target(*begin, g);
      cell::ptr to = g[sink]->cell(), from = g[source]->cell();
      cell_map[sink] = to;
      cell_map[source] = from;
      std::string to_port = g[*begin]->to_port();
      std::string from_port = g[*begin]->from_port();
      connections.push_back(boost::make_tuple(source, from_port, sink, to_port));
    }
    ar << cell_map;
    ar << connections;
  }

  template<class Archive>
  void
  plasm::load(Archive & ar, const unsigned int version)
  {
    serialization::cell_map_t cell_map;
    std::vector<serialization::connection_t> connections;
    ar >> cell_map;
    ar >> connections;
    for (size_t i = 0; i < connections.size(); i++)
    {
      cell::ptr from, to;
      from = cell_map[connections[i].get<0>()];
      to = cell_map[connections[i].get<2>()];
      std::string from_port, to_port;
      from_port = connections[i].get<1>();
      to_port = connections[i].get<3>();
      connect(from, from_port, to, to_port);
    }
  }
  #pragma GCC diagnostic ignored "-Wsign-compare"
  ECTO_INSTANTIATE_SERIALIZATION(plasm);

}

namespace boost
{
  namespace serialization
  {
    template<typename Archive>
    void
    serialize(Archive& ar,
              ecto::serialization::connection_t& c,
              const unsigned int version )
    {
      ar & c.get<0>();
      ar & c.get<1>();
      ar & c.get<2>();
      ar & c.get<3>();
    }
  }
}

