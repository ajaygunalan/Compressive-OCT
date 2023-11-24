#ifndef OCT_MSGS__VISIBILITY_CONTROL_H_
#define OCT_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OCT_MSGS_EXPORT __attribute__ ((dllexport))
    #define OCT_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define OCT_MSGS_EXPORT __declspec(dllexport)
    #define OCT_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef OCT_MSGS_BUILDING_LIBRARY
    #define OCT_MSGS_PUBLIC OCT_MSGS_EXPORT
  #else
    #define OCT_MSGS_PUBLIC OCT_MSGS_IMPORT
  #endif
  #define OCT_MSGS_PUBLIC_TYPE OCT_MSGS_PUBLIC
  #define OCT_MSGS_LOCAL
#else
  #define OCT_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define OCT_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define OCT_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define OCT_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OCT_MSGS_PUBLIC
    #define OCT_MSGS_LOCAL
  #endif
  #define OCT_MSGS_PUBLIC_TYPE
#endif
#endif  // OCT_MSGS__VISIBILITY_CONTROL_H_
// Generated 31-Jul-2023 17:17:27
// Copyright 2019-2020 The MathWorks, Inc.
