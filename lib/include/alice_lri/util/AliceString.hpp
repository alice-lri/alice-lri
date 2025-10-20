/**
 * @file AliceString.hpp
 * @brief String class for Alice LRI library, providing owning, mutable, null-terminated strings.
 */
#pragma once
#include <cstdint>

#include "alice_lri/ApiGuards.hpp"

namespace alice_lri {

    /**
     * @brief String class for Alice LRI library. Owns its data, is mutable, and always null-terminated.
     *
     * This class is a thin PIMPL wrapper around std::string, providing explicit memory management and ABI compatibility.
     */
    class ALICE_LRI_API AliceString {
    private:
        struct Impl; /**< Implementation details (opaque pointer to std::string). */
        Impl* impl = nullptr;

    public:
        /**
         * @brief Default constructor. Creates an empty string.
         */
        AliceString() noexcept;

        /**
         * @brief Construct from a null-terminated C string.
         * @param c_str Null-terminated C string (may be nullptr for empty).
         */
        explicit AliceString(const char* c_str) noexcept;

        /**
         * @brief Construct from a data pointer and length.
         * @param data Pointer to character data (may be nullptr for empty).
         * @param n Number of characters to copy from data.
         */
        AliceString(const char* data, uint64_t n) noexcept;

        /**
         * @brief Copy constructor. Deep copy of the string.
         * @param other String to copy from.
         */
        AliceString(const AliceString& other) noexcept;

        /**
         * @brief Copy assignment. Deep copy of the string.
         * @param other String to copy from.
         * @return Reference to this string.
         */
        AliceString& operator=(const AliceString& other) noexcept;

        /**
         * @brief Move constructor. Transfers ownership.
         * @param other String to move from.
         */
        AliceString(AliceString&& other) noexcept;

        /**
         * @brief Move assignment. Transfers ownership.
         * @param other String to move from.
         * @return Reference to this string.
         */
        AliceString& operator=(AliceString&& other) noexcept;

        /**
         * @brief Assign from a null-terminated C string.
         * @param c_str Null-terminated C string (may be nullptr for empty).
         * @return Reference to this string.
         */
        AliceString& operator=(const char* c_str) noexcept;

        /**
         * @brief Destructor. Releases owned memory.
         */
        ~AliceString() noexcept;

        /**
         * @brief Get the number of characters in the string.
         * @return String size (number of characters, not including null terminator).
         */
        [[nodiscard]] uint64_t size() const noexcept;

        /**
         * @brief Get a pointer to the null-terminated C string.
         * @return Pointer to null-terminated character data (never nullptr).
         */
        [[nodiscard]] const char* c_str() const noexcept;

        /**
         * @brief Check if the string is empty.
         * @return True if the string is empty.
         */
        [[nodiscard]] bool empty() const noexcept;

        /**
         * @brief Access character at index.
         * @param i Index of character (no bounds check).
         * @return Reference to character.
         */
        char& operator[](uint64_t i) noexcept;

        /**
         * @brief Access character at index (const).
         * @param i Index of character (no bounds check).
         * @return Const reference to character.
         */
        const char& operator[](uint64_t i) const noexcept;

        /**
         * @brief Iterator to beginning of string data.
         * @return Pointer to first character.
         */
        char* begin() noexcept { return const_cast<char*>(c_str()); }

        /**
         * @brief Iterator to end of string data.
         * @return Pointer one past the last character.
         */
        char* end() noexcept { return const_cast<char*>(c_str()) + size(); }

        /**
         * @brief Const iterator to beginning of string data.
         * @return Pointer to first character.
         */
        [[nodiscard]] const char* begin() const noexcept { return c_str(); }

        /**
         * @brief Const iterator to end of string data.
         * @return Pointer one past the last character.
         */
        [[nodiscard]] const char* end() const noexcept { return c_str() + size(); }

        /**
         * @brief Clear the string, making it empty.
         */
        void clear() noexcept;

        /**
         * @brief Reserve memory for at least n characters.
         * @param n Minimum capacity to reserve.
         */
        void reserve(uint64_t n) noexcept;

        /**
         * @brief Shrink capacity to fit the current size.
         */
        void shrink_to_fit() noexcept;

        /**
         * @brief Resize the string to n characters, filling with fill if needed.
         * @param n New size.
         * @param fill Character to use for new elements (default '\0').
         */
        void resize(uint64_t n, char fill = '\0') noexcept;

        /**
         * @brief Append a character to the end of the string.
         * @param ch Character to append.
         */
        void push_back(char ch) noexcept;

        /**
         * @brief Append a null-terminated C string.
         * @param c_str Null-terminated C string (may be nullptr for no-op).
         */
        void append(const char* c_str) noexcept;

        /**
         * @brief Append data from a pointer and length.
         * @param data Pointer to data (may be nullptr for no-op).
         * @param n Number of characters to append.
         */
        void append(const char* data, uint64_t n) noexcept;

        /**
         * @brief Append another AliceString.
         * @param other String to append.
         */
        void append(const AliceString& other) noexcept;
    };

}
