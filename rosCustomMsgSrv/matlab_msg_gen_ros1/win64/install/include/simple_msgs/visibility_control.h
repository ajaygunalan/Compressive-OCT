#ifndef SIMPLE_MSGS__VISIBILITY_CONTROL_H_
#define SIMPLE_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SIMPLE_MSGS_EXPORT __attribute__ ((dllexport))
    #define SIMPLE_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define SIMPLE_MSGS_EXPORT __declspec(dllexport)
    #define SIMPLE_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef SIMPLE_MSGS_BUILDING_LIBRARY
    #define SIMPLE_MSGS_PUBLIC SIMPLE_MSGS_EXPORT
  #else
    #define SIMPLE_MSGS_PUBLIC SIMPLE_MSGS_IMPORT
  #endif
  #define SIMPLE_MSGS_PUBLIC_TYPE SIMPLE_MSGS_PUBLIC
  #define SIMPLE_MSGS_LOCAL
#else
  #define SIMPLE_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define SIMPLE_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define SIMPLE_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define SIMPLE_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SIMPLE_MSGS_PUBLIC
    #define SIMPLE_MSGS_LOCAL
  #endif
  #define SIMPLE_MSGS_PUBLIC_TYPE
#endif
#endif  // SIMPLE_MSGS__VISIBILITY_CONTROL_H_
// Generated 15-Jan-2024 13:23:41
// Copyright 2019-2020 The MathWorks, Inc.
