/**
 * @file ApiGuards.hpp
 * @brief API guard macros and utilities for Alice LRI library.
 */
#pragma once
#if defined(ALICE_LRI_WHITE_BOX) && (ALICE_LRI_WHITE_BOX == 1)
    /**
     * @def ALICE_LRI_API
     * @brief API export/import macro for Alice LRI library (no-op for white box builds).
     */
    #define ALICE_LRI_API
#else
    #if defined(_WIN32)
        /**
         * @def ALICE_LRI_API
         * @brief API export/import macro for Alice LRI library (Windows).
         */
        #if defined(ALICE_LRI_EXPORTS)
            #define ALICE_LRI_API __declspec(dllexport)
        #else
            #define ALICE_LRI_API __declspec(dllimport)
        #endif
    #else
        /**
         * @def ALICE_LRI_API
         * @brief API export/import macro for Alice LRI library (non-Windows).
         */
        #define ALICE_LRI_API __attribute__((visibility("default")))
    #endif
#endif
