/**
 * @file Result.hpp
 * @brief Result type and error handling utilities for Alice LRI library.
 */
#pragma once
#include <new>
#include "alice_lri/ApiGuards.hpp"
#include "util/AliceString.hpp"

namespace alice_lri {


    /**
     * @brief Error codes for Alice LRI operations.
     */
    enum class ErrorCode {
        NONE = 0,              /**< No error. */
        MISMATCHED_SIZES,      /**< Input arrays have mismatched sizes. */
        EMPTY_POINT_CLOUD,     /**< Point cloud is empty. */
        RANGES_XY_ZERO,        /**< At least one point has a range of zero in the XY plane. */
        INTERNAL_ERROR,        /**< Internal error occurred. */
    };

    /**
     * @brief Get a human-readable error message for an ErrorCode.
     * @param code Error code.
     * @return Error message string.
     */
    AliceString ALICE_LRI_API errorMessage(ErrorCode code);

    /**
     * @brief Status of an operation, including error code and message.
     */
    struct ALICE_LRI_API Status {
        /** Error code. */
        ErrorCode code = ErrorCode::NONE;
        /** Error message. */
        AliceString message = AliceString();

        /**
         * @brief Build a successful status.
         * @return Status with no error.
         */
        static Status buildOk() noexcept { return {ErrorCode::NONE, AliceString()}; }
        /**
         * @brief Build an error status from code.
         * @param c Error code.
         * @return Status with error code and message.
         */
        static Status buildError(const ErrorCode c) noexcept { return {c, errorMessage(c)}; }
        /**
         * @brief Build an error status from code and custom message.
         * @param c Error code.
         * @param msg Custom message.
         * @return Status with error code and custom message.
         */
        static Status buildError(const ErrorCode c, const AliceString &msg) noexcept { return {c, msg}; }

        /**
         * @brief Check if status is OK (no error).
         * @return True if no error.
         */
        explicit operator bool() const noexcept { return code == ErrorCode::NONE; }
    };

    /**
     * @brief Result type for operations that may fail, containing either a value or a status.
     * @tparam T Type of the value.
     */
    template<class T>
    class Result {
    private:
        union {
            T value_;      /**< The value, if operation succeeded. */
            Status status_;/**< The status, if operation failed. */
        };
        bool hasValue_;    /**< True if value is present, false if status. */

    public:

        /**
         * @brief Construct a Result with a value.
         * @param v The value.
         */
        explicit Result(const T& v) noexcept : value_(v), hasValue_(true) {}
        /**
         * @brief Construct a Result with a value (move).
         * @param v The value (rvalue).
         */
        explicit Result(T&& v) noexcept : value_(static_cast<T&&>(v)), hasValue_(true) {}
        /**
         * @brief Construct a Result with a status.
         * @param s The status.
         */
        explicit Result(const Status& s) noexcept : status_(s), hasValue_(false) { }
        /**
         * @brief Construct a Result with a status (move).
         * @param s The status (rvalue).
         */
        explicit Result(Status&& s) noexcept : status_(static_cast<Status&&>(s)), hasValue_(false) { }

        /** Destructor. */
        ~Result() {
            if (hasValue_) {
                value_.~T();
            } else {
                status_.~Status();
            }
        }

        /** Copy constructor. */
        Result(const Result& o) : hasValue_(o.hasValue_) {
            if (hasValue_) {
                new(&value_) T(o.value_);
            } else {
                new(&status_) Status(o.status_);
            }
        }

        /** Copy assignment. */
        Result& operator=(const Result& o) {
            if (this == &o) {
                return *this;
            }
            this->~Result();
            new(this) Result(o);
            return *this;
        }

        /** Move constructor. */
        Result(Result&& o) noexcept : hasValue_(o.hasValue_) {
            if (hasValue_) {
                new(&value_) T(static_cast<T&&>(o.value_));
            } else {
                new(&status_) Status(static_cast<Status&&>(o.status_));
            }
        }

        /** Move assignment. */
        Result& operator=(Result&& o) noexcept {
            if (this == &o) {
                return *this;
            }
            this->~Result();
            new(this) Result(static_cast<T&&>(o));
            return *this;
        }

        /**
         * @brief Get the status (if not ok).
         * @return Status object.
         */
        [[nodiscard]] const Status& status() const {
            if (!hasValue_) {
                return status_;
            }

            static Status okStatus = Status::buildOk();
            return okStatus;
        }

        /**
         * @brief Check if the result is ok (has value).
         * @return True if value is present.
         */
        [[nodiscard]] bool ok() const noexcept { return hasValue_; }
        /**
         * @brief Get the value (lvalue).
         * @return Reference to value.
         */
        T& value() & { return value_;     }
        /**
         * @brief Get the value (const lvalue).
         * @return Const reference to value.
         */
        const T& value() const & { return value_; }
        /**
         * @brief Get the value (rvalue).
         * @return Rvalue reference to value.
         */
        T&& value() && { return static_cast<T&&>(value_); }

        /**
         * @brief Check if result is ok (implicit conversion).
         * @return True if value is present.
         */
        explicit operator bool() const noexcept { return ok(); }
        /**
         * @brief Dereference to value.
         * @return Reference to value.
         */
        T& operator*() { return value(); }
        /**
         * @brief Dereference to value (const).
         * @return Const reference to value.
         */
        const T& operator*() const { return value(); }
        /**
         * @brief Pointer access to value.
         * @return Pointer to value.
         */
        T* operator->() { return &value(); }
        /**
         * @brief Pointer access to value (const).
         * @return Const pointer to value.
         */
        const T* operator->() const { return &value(); }
    };
}
