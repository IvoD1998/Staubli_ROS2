#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define URDF_EXPORT __attribute__ ((dllexport))
    #define URDF_IMPORT __attribute__ ((dllimport))
  #else
    #define URDF_EXPORT __declspec(dllexport)
    #define URDF_IMPORT __declspec(dllimport)
  #endif
  #ifdef URDF_BUILDING_LIBRARY
    #define URDF_PUBLIC URDF_EXPORT
  #else
    #define URDF_PUBLIC URDF_IMPORT
  #endif
  #define URDF_PUBLIC_TYPE URDF_PUBLIC
  #define URDF_LOCAL
#else
  #define URDF_EXPORT __attribute__ ((visibility("default")))
  #define URDF_IMPORT
  #if __GNUC__ >= 4
    #define URDF_PUBLIC __attribute__ ((visibility("default")))
    #define URDF_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define URDF_PUBLIC
    #define URDF_LOCAL
  #endif
  #define URDF_PUBLIC_TYPE
#endif
