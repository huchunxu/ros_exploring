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
#include <boost/exception/all.hpp>

#if defined(ECTO_EXCEPTION_SHARED_POINTERS_ARE_CONST)
namespace boost {
  namespace exception_detail {

    struct shared_pointers_are_const : boost::exception_detail::error_info_container
    {
      virtual shared_ptr<error_info_base const> get( type_info_ const&) const 
      {
        return shared_ptr<error_info_base const>();
      }
    };
  }
}

#elif defined(ECTO_EXCEPTION_DIAGNOSTIC_IMPL_TAKES_CHARSTAR)
namespace boost {
  namespace exception_detail {

    void checkem(error_info_container_impl& ei)
    {
      const char* s;
      const char* rv = ei.diagnostic_information(s);
    }
  }
}

#elif defined(ECTO_EXCEPTION_RELEASE_RETURNS_VOID)
namespace boost {
  namespace exception_detail {

    struct release_returns_bool : boost::exception_detail::error_info_container
    {
      void release() const { }
    };
  }
}

#elif defined(ECTO_EXCEPTION_TAG_TYPE_NAME_RETURNS_STRING)
namespace boost {
  struct something { };
  template <> inline std::string tag_type_name<something>() { return "SOMETHING"; }
}
#elif defined(ECTO_EXCEPTION_TYPE_INFO_NESTED)
#include <boost/exception/detail/type_info.hpp>
void meh(::boost::exception_detail::type_info_& ti)
{
  const void * meh = ti.type_;
}
#elif defined(ECTO_EXCEPTION_CONTAINER_WITHOUT_CLONE)
#include <boost/exception/detail/type_info.hpp>
namespace boost {
  namespace exception_detail {

    struct fails_if_has_clone : boost::exception_detail::error_info_container
    {
      void clone() const { };
    };
  }
}

#else
#error ECTO_CHECK_TRY_COMPILE_ERROR
#endif

int main(int, char**) { }
