/******************************************************************************** 
 * MIT License
 * 
 * Copyright (c) 2020 Stereolabs
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ********************************************************************************/

#ifndef LIDAR_COMPONENTS__VISIBILITY_CONTROL_H_
#define LIDAR_COMPONENTS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LIDAR_COMPONENTS_EXPORT __attribute__ ((dllexport))
    #define LIDAR_COMPONENTS_IMPORT __attribute__ ((dllimport))
  #else
    #define LIDAR_COMPONENTS_EXPORT __declspec(dllexport)
    #define LIDAR_COMPONENTS_IMPORT __declspec(dllimport)
  #endif
  #ifdef LIDAR_COMPONENTS_BUILDING_DLL
    #define LIDAR_COMPONENTS_PUBLIC LIDAR_COMPONENTS_EXPORT
  #else
    #define LIDAR_COMPONENTS_PUBLIC LIDAR_COMPONENTS_IMPORT
  #endif
  #define LIDAR_COMPONENTS_PUBLIC_TYPE LIDAR_COMPONENTS_PUBLIC
  #define LIDAR_COMPONENTS_LOCAL
#else
  #define LIDAR_COMPONENTS_EXPORT __attribute__ ((visibility("default")))
  #define LIDAR_COMPONENTS_IMPORT
  #if __GNUC__ >= 4
    #define LIDAR_COMPONENTS_PUBLIC __attribute__ ((visibility("default")))
    #define LIDAR_COMPONENTS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LIDAR_COMPONENTS_PUBLIC
    #define LIDAR_COMPONENTS_LOCAL
  #endif
  #define LIDAR_COMPONENTS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_COMPONENTS__VISIBILITY_CONTROL_H_
