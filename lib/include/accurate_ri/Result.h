#pragma once
#include <new>
#include "accurate_ri/AliceString.h"

namespace accurate_ri {

    enum class ErrorCode {
        NONE = 0,
        MISMATCHED_SIZES,
        EMPTY_POINT_CLOUD,
        RANGES_XY_ZERO,
        INTERNAL_ERROR,
    };

    AliceString errorMessage(ErrorCode code);

    struct Status {
        ErrorCode code = ErrorCode::NONE;
        AliceString message = AliceString();

        static Status ok() noexcept { return {ErrorCode::NONE, AliceString()}; }
        static Status error(const ErrorCode c) noexcept { return {c, errorMessage(c)}; }
        static Status error(const ErrorCode c, const AliceString &msg) noexcept { return {c, msg}; }

        explicit operator bool() const noexcept { return code == ErrorCode::NONE; }
    };

    template<class T>
    class Result {
    private:
        union {
            T value_;
            Status status_;
        };
        bool hasValue_;

    public:

        explicit Result(const T& v) noexcept : value_(v), hasValue_(true) {}
        explicit Result(T&& v) noexcept : value_(static_cast<T&&>(v)), hasValue_(true) {}
        explicit Result(const Status& s) noexcept : status_(s), hasValue_(false) { }
        explicit Result(Status&& s) noexcept : status_(static_cast<Status&&>(s)), hasValue_(false) { }

        ~Result() {
            if (hasValue_) {
                value_.~T();
            } else {
                status_.~Status();
            }
        }

        Result(const Result& o) : hasValue_(o.hasValue_) {
            if (hasValue_) {
                new(&value_) T(o.value_);
            } else {
                new(&status_) Status(o.status_);
            }
        }

        Result& operator=(const Result& o) {
            if (this == &o) {
                return *this;
            }
            this->~Result();
            new(this) Result(o);
            return *this;
        }

        Result(Result&& o) noexcept : hasValue_(o.hasValue_) {
            if (hasValue_) {
                new(&value_) T(static_cast<T&&>(o.value_));
            } else {
                new(&status_) Status(static_cast<Status&&>(o.status_));
            }
        }

        Result& operator=(Result&& o) noexcept {
            if (this == &o) {
                return *this;
            }
            this->~Result();
            new(this) Result(static_cast<T&&>(o));
            return *this;
        }

        [[nodiscard]] const Status& status() const {
            if (!hasValue_) {
                return status_;
            }

            static Status okStatus = Status::ok();
            return okStatus;
        }

        [[nodiscard]] bool ok() const noexcept { return hasValue_; }
        T& value() & { return value_;     }
        const T& value() const & { return value_; }
        T&& value() && { return static_cast<T&&>(value_); }

        explicit operator bool() const noexcept { return ok(); }
        T& operator*() { return value(); }
        const T& operator*() const { return value(); }
        T* operator->() { return &value(); }
        const T* operator->() const { return &value(); }
    };
} // namespace accurate_ri
