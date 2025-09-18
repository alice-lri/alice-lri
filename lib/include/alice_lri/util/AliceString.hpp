#pragma once
#include <cstdint>

#include "alice_lri/ApiGuards.hpp"

namespace alice_lri {

    class ALICE_LRI_API AliceString {
    private:
        struct Impl;
        Impl* impl = nullptr;

    public:
        AliceString() noexcept;
        explicit AliceString(const char* c_str) noexcept;
        AliceString(const char* data, uint64_t n) noexcept;

        AliceString(const AliceString&) noexcept;
        AliceString& operator=(const AliceString&) noexcept;
        AliceString(AliceString&&) noexcept;
        AliceString& operator=(AliceString&&) noexcept;
        AliceString& operator=(const char* c_str) noexcept;
        ~AliceString() noexcept;

        [[nodiscard]] uint64_t size() const noexcept;
        [[nodiscard]] const char* c_str() const noexcept;
        [[nodiscard]] bool empty() const noexcept;

        char& operator[](uint64_t i) noexcept;
        const char& operator[](uint64_t i) const noexcept;

        char* begin() noexcept { return const_cast<char*>(c_str()); }
        char* end() noexcept { return const_cast<char*>(c_str()) + size(); }
        [[nodiscard]] const char* begin() const noexcept { return c_str(); }
        [[nodiscard]] const char* end() const noexcept { return c_str() + size(); }

        void clear() noexcept;
        void reserve(uint64_t n) noexcept;
        void shrink_to_fit() noexcept;
        void resize(uint64_t n, char fill = '\0') noexcept;

        void push_back(char ch) noexcept;
        void append(const char* c_str) noexcept;
        void append(const char* data, uint64_t n) noexcept;
        void append(const AliceString& other) noexcept;
    };

}
