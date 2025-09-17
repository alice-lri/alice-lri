#pragma once
#if defined(_WIN32)
  #if defined(ALICE_LRI_EXPORTS)
    #define ALICE_LRI_API __declspec(dllexport)
    #define ALICE_LRI_TEMPLATE_API
  #else
    #define ALICE_LRI_API __declspec(dllimport)
    #define ALICE_LRI_TEMPLATE_API extern
  #endif
#else
  #define ALICE_LRI_API __attribute__((visibility("default")))
  #define ALICE_LRI_TEMPLATE_API
#endif