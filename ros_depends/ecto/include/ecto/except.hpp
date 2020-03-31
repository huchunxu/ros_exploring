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

#define ECTO_EXCEPTION_TAG_NAMES                                        \
  (from_typename)(to_typename)(from_key)(to_key)(from_cell)             \
  (to_cell)(cpp_typename)(pyobject_repr)(actualtype_hint)(spore_typename) \
  (diag_msg)(actualkeys_hint)(tendril_key)(cell_name)(cell_type)(function_name)    \
  (hint)(which_tendrils)(prev_typename)(cur_typename)(type)             \
  (what)(when)

#define ECTO_EXCEPTIONS                                                 \
    (TypeMismatch)(ValueNone)(ValueRequired)(NonExistant)               \
    (FailedFromPythonConversion)(TendrilRedeclaration)(CellException)   \
    (NotConnected)(AlreadyConnected)(NullTendril)

// these are to speed up the preprocessor metaprogramming
# ifndef BOOST_PREPROCESSOR_CONFIG_LIMITS_HPP
# define BOOST_PREPROCESSOR_CONFIG_LIMITS_HPP
#
# define ECTO_PP_ITERLIMIT 22
# define BOOST_PP_LIMIT_MAG ECTO_PP_ITERLIMIT
# define BOOST_PP_LIMIT_TUPLE 25
# define BOOST_PP_LIMIT_DIM 3
# define BOOST_PP_LIMIT_REPEAT ECTO_PP_ITERLIMIT
# define BOOST_PP_LIMIT_WHILE ECTO_PP_ITERLIMIT
# define BOOST_PP_LIMIT_FOR ECTO_PP_ITERLIMIT
# define BOOST_PP_LIMIT_ITERATION ECTO_PP_ITERLIMIT
# define BOOST_PP_LIMIT_ITERATION_DIM 2
# define BOOST_PP_LIMIT_SEQ ECTO_PP_ITERLIMIT
# define BOOST_PP_LIMIT_SLOT_SIG 10
# define BOOST_PP_LIMIT_SLOT_COUNT 5
#
# endif

#include <Python.h>
#include <boost/version.hpp>
#include <stdexcept>
#include <string>
#include <map>
#include <ecto/config.hpp>
#include <ecto/util.hpp>
#include <boost/optional.hpp>

#include <boost/exception/exception.hpp>
#include <boost/exception/info.hpp>

#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/seq/for_each.hpp>

#pragma once
namespace ecto
{
  namespace except
  {
    namespace detail {
      template <class T>
      struct wrap { };
    }

    class error_info_container_impl
      : public ::boost::exception_detail::error_info_container
    {
      typedef ::boost::exception_detail::type_info_ type_info_;
      typedef ::boost::exception_detail::error_info_base error_info_base;

      //
      //  hacks for boost::exception implementation details that are
      //  moving around.
      //
#if defined(ECTO_EXCEPTION_SHARED_POINTERS_ARE_CONST)
      typedef boost::shared_ptr<error_info_base const> error_info_base_ptr;
      typedef void diagnostic_information_arg_t;
#else
      typedef boost::shared_ptr<error_info_base> error_info_base_ptr;
      typedef const char* diagnostic_information_arg_t;
#endif

    public:

      error_info_container_impl();

      ~error_info_container_impl() throw();

      void
      set(error_info_base_ptr const & x,
          type_info_ const & typeid_);

      error_info_base_ptr
      get( type_info_ const & ti ) const;

      char const * diagnostic_information(
#if defined(ECTO_EXCEPTION_DIAGNOSTIC_IMPL_TAKES_CHARSTAR)
                                          char const*
#endif
                                          ) const;
    private:

      friend class boost::exception;

      typedef std::map<std::string, error_info_base_ptr>
      error_info_map;
      error_info_map info_;
      mutable std::string diagnostic_info_str_;
      mutable int count_;

      void add_ref() const;
#if defined(ECTO_EXCEPTION_RELEASE_RETURNS_VOID)
      void release() const;
#else
      bool release() const;
#endif
#if defined(ECTO_EXCEPTION_HAS_CLONE)
      ::boost::exception_detail::refcount_ptr< ::boost::exception_detail::error_info_container> clone() const;
#endif

    };

    struct EctoException : virtual std::exception, virtual boost::exception
    {
      EctoException();
      virtual const char* what() const throw();
    };

    // here, what() actually returns the type name,  the errinfo tag stuff
    // is used for the more illuminating error information
#define ECTO_DECLARE_EXCEPTION(r, data, T)                              \
    struct T : virtual EctoException                                    \
    {                                                                   \
      const char* what() const throw();                                 \
    };

    BOOST_PP_SEQ_FOR_EACH(ECTO_DECLARE_EXCEPTION, ~, ECTO_EXCEPTIONS);

    std::string diagnostic_string(const EctoException&);

    boost::optional<std::string> diagnostic_string(const EctoException& e,
                                                   const std::string& tag);

#define ECTO_EXCEPTION_TAG_DECL(r, data, NAME)                          \
    struct BOOST_PP_CAT(tag_, NAME);                                    \
    typedef ::boost::error_info<detail::wrap<BOOST_PP_CAT(tag_, NAME)>,         \
                                std::string> NAME;                      \

    BOOST_PP_SEQ_FOR_EACH(ECTO_EXCEPTION_TAG_DECL, ~, ECTO_EXCEPTION_TAG_NAMES);

  }
}

namespace boost {

#if defined(ECTO_EXCEPTION_TAG_TYPE_NAME_RETURNS_STRING)
#  define ECTO_EXCEPTION_TAG_TYPE_NAME_RETURN_T std::string
#else
#  define ECTO_EXCEPTION_TAG_TYPE_NAME_RETURN_T const char*
#endif

#define ECTO_EXCEPTION_TAG_TYPE_NAME_DECL(r, data, NAME)                \
  template <> inline                                                    \
  ECTO_EXCEPTION_TAG_TYPE_NAME_RETURN_T                                 \
  tag_type_name< ::ecto::except::detail::wrap< BOOST_PP_CAT(::ecto::except::tag_, NAME)> >() { \
    return BOOST_PP_STRINGIZE(NAME);                                    \
  }
  BOOST_PP_SEQ_FOR_EACH(ECTO_EXCEPTION_TAG_TYPE_NAME_DECL, ~, ECTO_EXCEPTION_TAG_NAMES);

  template <class E,class Tag,class T>
#if ((BOOST_VERSION / 100) % 1000) <= 42
  E const &
#else
  typename enable_if<exception_detail::derives_boost_exception<E>,E const &>::type
#endif
  operator<<( E const & x,
              error_info< ::ecto::except::detail::wrap<Tag>, T> const & v );

}

