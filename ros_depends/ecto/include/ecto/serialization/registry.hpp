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

#if BOOST_VERSION <= 104000
#pragma GCC diagnostic ignored "-Wsign-compare"
// TODO EAR meh noisy -- We should make a portable stable archive type.
//#pragma message "Ignoring signed-unsigned comparison in boost::serialization in 1.40"
#endif

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <ecto/util.hpp>
#include <ecto/tendril.hpp>
#include <boost/tuple/tuple.hpp>
#include <map>
#include <ecto/log.hpp>
namespace ecto
{
  namespace serialization
  {
    template<typename T, typename Archive>
    struct writer_
    {
      typedef T value_type;
      void
      operator()(Archive& ar, const tendril& t) const
      {
        ECTO_LOG_DEBUG("%s", "");
        ar << t.get<T>();
      }
    };

    template<typename T, typename Archive>
    struct reader_
    {
      typedef T value_type;
      void
      operator()(Archive& ar, tendril& t) const
      {
        ECTO_LOG_DEBUG("%s", "");
        if (!t.is_type<T>())
          t << tendril(T(), ""); //don't want to lose docs.
        ar >> t.get<T>();
      }
    };

    template<typename Archive>
    struct registry: boost::noncopyable
    {
      typedef boost::function<void(Archive&, tendril&)> serial_fn_t;
      typedef std::map<std::string, serial_fn_t> serial_map_t;

      template<typename Serializer>
      void
      add(const Serializer& s)
      {
        ECTO_LOG_DEBUG("%s", "");
        typedef typename Serializer::value_type value_type;
        const std::string& name = name_of<value_type>();
        serial_fn_t fnc = serial_fn_t(s);
        add(name, fnc);
      }

      void
      add(const std::string& name, serial_fn_t fnc);
      void
      serialize(const std::string& key, Archive& ar, tendril& t) const;

      serial_map_t serial_map;

      static registry<Archive>& instance();

    private:
      registry();
    };

    extern template struct registry<boost::archive::binary_oarchive> ;
    extern template struct registry<boost::archive::binary_iarchive> ;

    typedef registry<boost::archive::binary_oarchive> registry_binary_oa;
    typedef registry<boost::archive::binary_iarchive> registry_binary_ia;


    template<typename T>
    struct register_serializer: boost::noncopyable
    {
      typedef writer_<T, boost::archive::binary_oarchive> writer_binary_oa;
      typedef reader_<T, boost::archive::binary_iarchive> reader_binary_ia;
    private:
      static const register_serializer instance;
      register_serializer()
      {
        serialization::registry_binary_oa::instance().add(writer_binary_oa());
        serialization::registry_binary_ia::instance().add(reader_binary_ia());
      }
    };

    template<typename T>
    const register_serializer<T> register_serializer<T>::instance;

  }

}

#define ECTO_REGISTER_SERIALIZERS(Type)       \
namespace ecto{                               \
  namespace serialization{                    \
    template struct register_serializer<Type>; \
  }                                           \
}

#define ECTO_INSTANTIATE_SERIALIZATION(T)                               \
  template void T::serialize(boost::archive::binary_oarchive&, const unsigned int); \
  template void T::serialize(boost::archive::binary_iarchive&, const unsigned int);

