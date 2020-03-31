#pragma once

#include <ecto/abi.hpp>
#include <string>
#include <stdint.h>

#define ECTO_COMMON_TYPES \
  (char)(int)(long)(float)(double)(uint8_t)(uint16_t)(uint32_t)(uint64_t)

namespace ecto
{
  template<typename T>
  struct bounded
  {
    bounded(const T& value, const T& min, const T& max);

    explicit bounded(const T& value);

    bounded&
    operator=(const T& value);

    void set(const T& value);

    bool
    check(const T& value) const;

    std::string
    bounds() const;

    operator T() const;

    T value, min, max;
    bool has_bounds;
  };

}
