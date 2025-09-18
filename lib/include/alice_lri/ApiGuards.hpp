#pragma once
#if defined(ALICE_LRI_WHITE_BOX) && (ALICE_LRI_WHITE_BOX == 1)
    #define ALICE_LRI_API
#else
    #if defined(_WIN32)
        #if defined(ALICE_LRI_EXPORTS)
            #define ALICE_LRI_API __declspec(dllexport)
        #else
            #define ALICE_LRI_API __declspec(dllimport)
        #endif
    #else
        #define ALICE_LRI_API __attribute__((visibility("default")))
    #endif
#endif
