#pragma once
#if defined(_WIN32)
  #if defined(MYLIB_BUILD)
    #define ACCURATE_RI_API __declspec(dllexport)
  #else
    #define ACCURATE_RI_API __declspec(dllimport)
  #endif
#else
  #define ACCURATE_RI_API __attribute__((visibility("default")))
#endif