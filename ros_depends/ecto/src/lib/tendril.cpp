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
#include <ecto/tendril.hpp>
#include <boost/python.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ecto/python.hpp>
namespace ecto
{
  using namespace except;

  tendril::tendril()
    : doc_()
    , flags_()
    , converter(&ConverterImpl<none>::instance)
  {
    set_holder<none>(none());
  }

  tendril::tendril(const tendril& rhs)
    : holder_(rhs.holder_)
    , type_ID_(rhs.type_ID_)
    , doc_(rhs.doc_)
    , flags_(rhs.flags_)
    , converter(rhs.converter)
  { }

  tendril& tendril::operator=(const tendril& rhs)
  {
    if (this == &rhs)
      return *this;
    copy_holder(rhs);
    doc_ = rhs.doc_;
    flags_ = rhs.flags_;
    converter = rhs.converter;
    return *this;
  }

  tendril::~tendril(){ }

  ecto::tendril& tendril::operator<<(const tendril& rhs)
  {
    if (this == &rhs)
      return *this;
    if (is_type<none>() || same_type(rhs))
    {
      copy_holder(rhs);
    }
    else
    {
      enforce_compatible_type(rhs);
      if (rhs.is_type<none>())
      {
        BOOST_THROW_EXCEPTION(ecto::except::ValueNone());
      }
      else if (rhs.is_type<boost::python::object>())
      {
        ECTO_SCOPED_CALLPYTHON();
        *this << rhs.get<boost::python::object>();
      }
      else if (is_type<boost::python::object>())
      {
        ECTO_SCOPED_CALLPYTHON();
        (*rhs.converter)(*boost::unsafe_any_cast<boost::python::object>(&holder_), rhs);
      }
    }
    user_supplied(true);
    return *this;
  }


  void tendril::set_doc(const std::string& doc_str)
  {
    doc_ = doc_str;
  }

  void tendril::notify()
  {
    if (dirty())
    {
      jobs_(*this);
    }
    dirty(false);
  }

  std::string
  tendril::doc() const
  {
    return doc_;
  }

  std::string
  tendril::type_name() const
  {
   return type_ID_;
  }

  bool
  tendril::required() const
  {
    return flags_[REQUIRED];
  }

  void
  tendril::required(bool b)
  {
    flags_[REQUIRED] = b;
  }

  bool
  tendril::user_supplied() const
  {
    return flags_[USER_SUPPLIED];
  }

  void
  tendril::user_supplied(bool b)
  {
    flags_[USER_SUPPLIED] = b;
  }

  bool
  tendril::has_default() const
  {
    return flags_[DEFAULT_VALUE];
  }

  bool
  tendril::dirty() const
  {
    return flags_[DIRTY];
  }

  void
  tendril::dirty(bool dirty)
  {
    flags_[DIRTY] = dirty;
  }

  bool
  tendril::same_type(const tendril& rhs) const
  {
    return rhs.type_name() == type_name();
  }

  bool
  tendril::compatible_type(const tendril& rhs) const
  {
    if (same_type(rhs))
      return true;
    return is_type<none>() || rhs.is_type<none>() || is_type<boost::python::object>()
           || rhs.is_type<boost::python::object>();
  }

  void
  tendril::enforce_compatible_type(const tendril& rhs) const
  {
    if (!compatible_type(rhs))
    {
      BOOST_THROW_EXCEPTION(except::TypeMismatch() << from_typename(type_name())
                            << to_typename(rhs.type_name()));
        ;
    }
  }

  void tendril::copy_holder(const tendril& rhs)
  {
    holder_ = rhs.holder_;
    type_ID_ = rhs.type_ID_;
    converter = rhs.converter;
  }

  void tendril::operator<<(const boost::python::object& obj)
  {
    if (is_type<boost::python::object>())
      {
        holder_ = obj;
      }
    else if (is_type<none>())
      {
        set_holder(obj);
      }
    else
      (*converter)(*this, obj);
  }

  void
  operator>>(const tendril_ptr& rhs, boost::python::object& obj)
  {
    if (!rhs)
      BOOST_THROW_EXCEPTION(except::NullTendril()
                            << except::from_typename("(null)")
                            << except::to_typename("(python object)"));
    *rhs >> obj;
  }

  // e.g.  ISO C++ says that these are ambiguous, even...
  void
  operator>>(const tendril_cptr& rhs, boost::python::object& obj)
  {
    if (!rhs)
      BOOST_THROW_EXCEPTION(except::NullTendril()
                            << except::from_typename("(null)")
                            << except::to_typename("(python object)"));
    *rhs >> obj;
  }
  void
  operator<<(const tendril_ptr& lhs, const tendril_ptr& rhs)
  {
    if (!lhs)
      BOOST_THROW_EXCEPTION(except::NullTendril()
                            << except::to_typename("(null)")
                            << except::from_typename(rhs ? rhs->type_name() : "(null)"));
    if (!rhs)
      BOOST_THROW_EXCEPTION(except::NullTendril()
                            << except::to_typename(lhs->type_name())
                            << except::from_typename("(null)"));
    *lhs << *rhs;
  }

  void
  operator<<(const tendril_ptr& lhs, const tendril_cptr& rhs)
  {
    if (!lhs)
      BOOST_THROW_EXCEPTION(except::NullTendril()
                            << except::to_typename("(null)")
                            << except::from_typename(rhs->type_name()));
    if (!rhs)
      BOOST_THROW_EXCEPTION(except::NullTendril()
                            << except::to_typename(lhs->type_name())
                            << except::from_typename("(null)"));
    *lhs << *rhs;
  }


  const tendril::empty_t tendril::empty = tendril::empty_t();

  namespace registry
  {
    namespace tendril
    {
      typedef std::map<std::string, ecto::tendril> tr_t;
      tr_t tr;

      tendril_ptr pre_reg[] =
      { ecto::make_tendril<int>(), //
        ecto::make_tendril<float>(), //
        ecto::make_tendril<double>(), //
        ecto::make_tendril<unsigned>(), //
        ecto::make_tendril<unsigned long>(), //
        ecto::make_tendril<bool>(), //
        ecto::make_tendril<std::string>(), //
        ecto::make_tendril<std::vector<int> >(), //
        ecto::make_tendril<std::vector<float> >(), //
        ecto::make_tendril<std::vector<double> >(), //
        ecto::make_tendril<boost::posix_time::ptime>() };

      bool
      add(const ecto::tendril& t)
      {
        bool inserted;
        tr_t::iterator it;
        boost::tie(it, inserted) = tr.insert(std::make_pair(t.type_name(), t));
        if (t.type_name() == name_of<std::string>()) {
          tr.insert(std::make_pair("std::string", t));
          tr.insert(std::make_pair("std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >", t));
        }
        return inserted;
      }
      const ecto::tendril&
      get(const std::string& type_name)
      {
        tr_t::const_iterator it = tr.find(type_name);
        if (it == tr.end())
          BOOST_THROW_EXCEPTION(
              except::TypeMismatch() << except::type(type_name) << except::what("Type has not been registered!"));
        return it->second;
      }
      struct first{
        template<typename T>
        typename T::first_type operator()(T pair){
          return pair.first;
        }
      };
      std::vector<std::string> type_names(){
        std::vector<std::string> r;
        std::transform(tr.begin(), tr.end(), std::back_inserter(r), first());
        return r;
      }
    }
  }
}

