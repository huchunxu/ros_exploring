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
#include <iostream>
#include <typeinfo>
#include <string>

/* Cmake will define MyLibrary_EXPORTS on Windows when it
configures to build a shared library. If you are going to use
another build system on windows or create the visual studio
projects by hand you need to define MyLibrary_EXPORTS when
building a DLL on windows.
*/
// We are using the Visual Studio Compiler and building Shared libraries

#if defined (_WIN32) 
  #if defined(ecto_cpp_EXPORTS)
    #define  ECTO_EXPORT __declspec(dllexport)
  #else
    #define  ECTO_EXPORT __declspec(dllimport)
  #endif /* MyLibrary_EXPORTS */
#else /* defined (_WIN32) */
 #define ECTO_EXPORT
#endif

namespace ecto
{
  /**
   * \brief Get the unmangled type name of a type_info object.
   * @param ti The type_info to look up unmangled name for.
   * @return The unmangled name. e.g. cv::Mat or pcl::PointCloud<pcl::PointXYZ>
   */
  ECTO_EXPORT const std::string& name_of(const std::type_info &ti);

  /**
   * \brief Demangle the given name.
   */
  ECTO_EXPORT const std::string& name_of(const std::string& name);

  ECTO_EXPORT std::string symbolic_name_of(const std::string& name);

  /**
   * \brief Get the unmangled type name of a type.
   * @tparam T the type that one wants a name for.
   * @return The unmangled name of the given type.
   */
  template<typename T>
  const std::string& name_of()
  {
    static const std::string& name_cache =  name_of(typeid(T));
    return name_cache;
  }

  template<typename T>
  const std::string& symbolic_name_of()
  {
    static const std::string name_cache = symbolic_name_of(name_of<T>());
    return name_cache;
  }
}

