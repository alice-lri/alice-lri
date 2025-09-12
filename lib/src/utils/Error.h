#pragma once
#include <stdexcept>
#include <string>
#include "accurate_ri/Result.h"

namespace accurate_ri {
    class DataValidationError final : public std::domain_error {
    private:
        ErrorCode errorCode;

    public:
        explicit DataValidationError(const ErrorCode code) : std::domain_error(std::string(errorMessage(code).c_str())),
            errorCode(code) {}

        [[nodiscard]] ErrorCode code() const noexcept {
            return errorCode;
        }
    };
}