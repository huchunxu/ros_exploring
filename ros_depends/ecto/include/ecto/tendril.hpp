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
#include <boost/shared_ptr.hpp>
#include <boost/function/function1.hpp>
#ifndef BOOST_SIGNALS2_MAX_ARGS
  #define BOOST_SIGNALS2_MAX_ARGS 3
#endif
#include <boost/signals2.hpp>
#include <boost/any.hpp>

#include <ecto/forward.hpp>

#include <ecto/util.hpp> //name_of
#include <ecto/except.hpp>

#include <ecto/python.hpp>
#include <ecto/python/repr.hpp>

#include <vector>
#include <string>
#include <bitset>

namespace ecto
{
  namespace registry{
    namespace tendril{
      bool add(const ecto::tendril& t);
      const ecto::tendril& get(const std::string& type_name);
      std::vector<std::string> type_names();
      template<typename T>
      struct entry{
        entry(const ecto::tendril& t)
        {
          add(t);
        }
      };
      template<typename T>
      void add(const ecto::tendril& t){
        static entry<T> e(t);
      }
    }
  }
  /**
   * \brief A tendril is the slender, winding organ of the
   * ecto::cell that gives it its awesome type erasure and uber
   * flexibility.
   *
   * Each tendril is a type erasing holder for any instance of any type,
   * and allows introspection including its value, type, and doc string.
   *
   * The tendril operates as a value holder, so treat it as such. If you would like to pass it around without copies,
   * construct a pointer to tendril, perhaps with the make_tendril<T>() function.
   *
   * Items held by the tendril must be copy constructible and copiable.
   */

  class ECTO_EXPORT tendril
  {
  public:
    typedef boost::function1<void, tendril&> TendrilJob;

    enum {
      DEFAULT_VALUE=0,
      DIRTY,
      USER_SUPPLIED,
      REQUIRED,
      OPTIONAL,
      N_FLAGS
    };

    struct empty_t { };
    const static empty_t empty;

    /**
     * \brief Creates a tendril that is initialized with the
     * tendril::none type. This should be fairly cheap.
     */
    tendril();

    ~tendril();

    tendril(const tendril& rhs);
    tendril& operator=(const tendril& rhs);

    /**
     * \brief A convenience constructor for creating a tendril
     * that holds the given type.
     * @tparam T The type to hide in this tendril
     * @param t default value for t
     * @param doc a documentation string
     */
    template <typename T>
    tendril (const T& t, const std::string& doc)
      : flags_()
      , converter(&ConverterImpl<T>::instance)
    {
      flags_[DEFAULT_VALUE]=true;
      set_holder<T>(t);
      set_doc(doc);
    }

    /**
     * \brief This is an unmangled type name for what ever tendril is
     * holding.
     *
     * @return the unmangled name, e.g. "cv::Mat", or
     * "pcl::PointCloud<pcl::PointXYZ>"
     */
    std::string
    type_name() const;
    const char* type_id() const {return type_ID_;}

    /**
     * \brief A doc string for this tendril, "foo is for the input
     * and will be mashed with spam."
     * @return A very descriptive human readable string of whatever
     * the tendril is holding on to.
     */
    std::string
    doc() const;

    /**
     * \brief The doc for this tendril is runtime defined, so you may want to update it.
     * @param doc_str A human readable description of the tendril.
     */
    void
    set_doc(const std::string& doc_str);

    /**
     * \brief This sets the default value of the tendril. This is a
     * @param val
     */
    template<typename T>
    void
    set_default_val(const T& val = T())
    {
      enforce_type<T>();
      flags_[DEFAULT_VALUE] = true;
      set_holder<T>(val);
    }

    void required(bool b);

    bool required() const;

    /**
     * Given T this will get the type from the tendril, also enforcing type with an exception.
     * @return a const reference to the value of the tendril (no copies)
     */
    template<typename T>
    inline const T&
    get() const
    {
      enforce_type<T>();
      return unsafe_get<T>();
    }

    template<typename T>
    inline T&
    get()
    {
      enforce_type<T>();
      return unsafe_get<T>();
    }

    template <typename T>
    void
    operator<<(const T& val)
    {
      if(is_type<none>()) //handle none case.
      {
        set_holder<T>(val);
      }else
      {
        //throws on failure
        get<T>() = val;
      }
    }

    void operator<<(const boost::python::object& obj);

    tendril& operator<<(const tendril& rhs);

    /**
     * \brief runtime check if the tendril is of the given type.
     * @return true if it is the type.
     */
    template<typename T>
    bool
    is_type() const
    {
      return name_of<T>() == type_name();
    }

    /**
     * \brief Test if the given tendril is the same type as this one
     * @param rhs The tendril to test against.
     * @return true if they are the same type.
     */
    bool
    same_type(const tendril& rhs) const;

    bool
    compatible_type(const tendril& rhs) const;

    void
    enforce_compatible_type(const tendril& rhs) const;

    /**
     * \brief runtime check if the tendril is of the given type, this will throw.
     */
    template<typename T>
    inline void
    enforce_type() const
    {
      if (!is_type<T>())
        BOOST_THROW_EXCEPTION(except::TypeMismatch()
                              << except::from_typename(type_name())
                              << except::to_typename(name_of<T>()));
    }

    //! The value that this tendril holds was supplied by the user at some point.
    bool
    user_supplied() const;

    void user_supplied(bool v);

    //! The tendril was initialized with default value.
    bool
    has_default() const;

    //! A none type for tendril when the tendril is uninitialized.
    struct none {
      none& operator=(const none&) { return *this; }
      const none& operator=(const none&) const { return *this; } // funny const assignment operator
      friend bool operator==(const none&, const none&) { return true; }
      friend std::ostream& operator<<(std::ostream& os, const none&)
      {
        os << "ecto::tendril::none"; return os;
      }
    };

    template<typename T>
    struct Caller
    {
      typedef typename boost::function1<void, T> CbT;
      Caller(CbT cb)
        : cb(cb)
      { }

      void
      operator()(tendril& t)
      {
        cb(t.get<T>());
      }
      CbT cb;
    };

    template<typename Signature>
    boost::signals2::connection connect(Signature slot)
    {
      return jobs_.connect(slot);
    }

    /**
     * Register a typed callback with the tendril... Will throw on wrong type.
     * @param cb Will be called by the notify function, if the tendril is dirty.
     * @return  this
     */
    template<typename T>
    tendril&
    set_callback(typename boost::function1<void, T> cb)
    {
      typedef Caller<T> CallerT;
      enforce_type<T>();
      connect(CallerT(cb));
      return *this;
    }

    //! Notify the callback, only if this is dirty.
    void
    notify();

    //! The tendril has likely been modified since the last time that notify has beend called.
    //! This gets unset after its changed-callbacks have fired
    bool
    dirty() const;

    //! Set the tendril dirty, implying that the value has changed.
    void
    dirty(bool);

  private:

    template<typename T>
    inline const T& unsafe_get() const
    {
      return *boost::unsafe_any_cast<const T>(&holder_);
    }

    template<typename T>
    inline T& unsafe_get()
    {
      return *boost::unsafe_any_cast<T>(&holder_);
    }

    struct Converter
    {
      virtual void operator()(tendril& t, const boost::python::object& o) const = 0;
      virtual void operator()(boost::python::object& o, const tendril& t) const = 0;
    };

    template <typename T,  typename _=void>
    struct ConverterImpl : Converter
    {
      static ConverterImpl<T, _> instance;
      void
      operator()(tendril& t, const boost::python::object& obj) const
      {
        ECTO_SCOPED_CALLPYTHON();
        boost::python::extract<T> get_T(obj);
        if (get_T.check())
          t << get_T();
        else
          BOOST_THROW_EXCEPTION(except::FailedFromPythonConversion()
                                << except::pyobject_repr(ecto::py::repr(obj))
                                << except::cpp_typename(t.type_name()));
      }

      void
      operator()(boost::python::object& o, const tendril& t) const
      {
        ECTO_SCOPED_CALLPYTHON();
        const T& v = t.get<T>();
        boost::python::object obj(v);
        o = obj;
      }
    };

    template <typename _>
    struct ConverterImpl<none, _> : Converter
    {
      static ConverterImpl<none, _> instance;
      void
      operator()(tendril& t, const boost::python::object& obj) const
      {
        t << obj;
      }

      void
      operator()(boost::python::object& o, const tendril& t) const
      {
        ECTO_SCOPED_CALLPYTHON();
        o = boost::python::object();
      }
    };

    template <typename T>
    void set_holder(const T& t = T())
    {
      holder_ = t;
      type_ID_ = name_of<T>().c_str();
      converter = &ConverterImpl<T>::instance;
      registry::tendril::add<T>(*this);
    }
    void copy_holder(const tendril& rhs);

    boost::any holder_;
    const char* type_ID_;
    std::string doc_;
    std::bitset<N_FLAGS> flags_;
    typedef boost::signals2::signal<void(tendril&)> job_signal_t;
    job_signal_t jobs_;
    Converter* converter;

  public:


    template <typename T>
    void operator>>(T& val) const
    {
      val = get<T>();
    }

    void operator>>(boost::python::object& obj) const
    {
      (*converter)(obj, *this);
    }

    void operator>>(const tendril_ptr& rhs) const
    {
      *rhs << *this;
    }

    void operator>>(tendril_ptr& rhs) const
    {
      *rhs << *this;
    }

    void operator>>(tendril& rhs) const
    {
      rhs << *this;
    }

    template<typename T>
    friend void
    operator>>(const tendril_cptr& rhs, T& val)
    {
      if (!rhs)
        BOOST_THROW_EXCEPTION(except::NullTendril()
                              << except::from_typename("(null)")
                              << except::to_typename(name_of<T>()));
      *rhs >> val;
    }

    // This is to avoid ambiguous conversion due to boost python funkiness.
    // e.g.  ISO C++ says that these are ambiguous, even...
    friend void
      operator>>(const tendril_ptr& rhs, boost::python::object& obj);

    // This is to avoid ambiguous conversion due to boost python funkiness.
    // e.g.  ISO C++ says that these are ambiguous, even...
    friend void
      operator>>(const tendril_cptr& rhs, boost::python::object& obj);

    template<typename T>
    friend void
    operator<<(const tendril_ptr& lhs, const T& rhs)
    {
      if (!lhs)
        BOOST_THROW_EXCEPTION(except::NullTendril()
                              << except::to_typename("(null)")
                              << except::from_typename(name_of<T>()));
      *lhs << rhs;
    }

    friend void
      operator<<(const tendril_ptr& lhs, const tendril_ptr& rhs);

    friend void
      operator<<(const tendril_ptr& lhs, const tendril_cptr& rhs);


    template <typename Archive>
      void serialize(Archive& ar, const unsigned int);

    //my version of clang
    //can't handle the friend impl inside the class.
    template <typename T>
    friend tendril_ptr make_tendril();

  };

  template <typename T>
  tendril_ptr make_tendril()
  {
    tendril_ptr t(new tendril());
    t->set_holder<T>();
    return t;
  }

  template <typename T, typename _>
  tendril::ConverterImpl<T,_>
  tendril::ConverterImpl<T,_>::instance;

  template <typename _>
  tendril::ConverterImpl<tendril::none,_>
  tendril::ConverterImpl<tendril::none,_>::instance;

}

