#pragma once

#include <ecto/abi.hpp>
#include <ecto/parameters.hpp>

#include <stdexcept>
#include <exception>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

namespace ecto
{
  template<typename T>
  bounded<T>::bounded(const T& value, const T& min, const T& max)
      :
        min(min),
        max(max),
        has_bounds(true)
  {
    *this = value;
  }

  template<typename T>
  bounded<T>::bounded(const T& value)
      :
        value(value),
        min(),
        max(),
        has_bounds(false)
  {
  }
  template<typename T>
  void
  bounded<T>::set(const T& value)
  {
    if (!check(value))
      throw std::runtime_error(
          "Bad bounds! " + boost::lexical_cast<std::string>(value) + " is not within: " + bounds());
    this->value = value;
  }

  template<typename T>
  bounded<T>&
  bounded<T>::operator=(const T& value)
  {
    set(value);
    return *this;
  }

  template<typename T>
  bool
  bounded<T>::check(const T& value) const
  {
    if (!has_bounds)
      return true;
    if (min >= value || max <= value)
      return false;
    return true;
  }

  template<typename T>
  std::string
  bounded<T>::bounds() const
  {
    return boost::str(
        boost::format("(%s,%s)") % boost::lexical_cast<std::string>(min) % boost::lexical_cast<std::string>(max));
  }

  template<typename T>
  bounded<T>::operator T() const
  {
    return value;
  }
}
