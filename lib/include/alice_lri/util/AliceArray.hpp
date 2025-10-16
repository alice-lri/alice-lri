/**
 * @file AliceArray.hpp
 * @brief Array utility templates and functions for Alice LRI library.
 */
#pragma once
#include <cstdint>
#include <cstdlib>
#include <initializer_list>

namespace alice_lri {
    /**
     * @brief Dynamic array implementation for Alice LRI library.
     * @tparam T Element type.
     */
    template<class T>
    class AliceArray {
    private:
        T* data_ = nullptr;        /**< Pointer to array data. */
        uint64_t size_ = 0;        /**< Number of elements. */
        uint64_t capacity_ = 0;    /**< Allocated capacity. */
        
        /**
         * @brief Grow the capacity to at least min_capacity.
         * @param min_capacity Minimum required capacity.
         */
        void grow_capacity(uint64_t min_capacity) noexcept {
            uint64_t new_capacity = capacity_ == 0 ? 1 : capacity_;
            while (new_capacity < min_capacity) {
                new_capacity *= 2;
            }
            
            T* new_data = static_cast<T*>(malloc(new_capacity * sizeof(T)));
            if (!new_data) return;

            for (uint64_t i = 0; i < size_; ++i) {
                new (new_data + i) T(static_cast<T&&>(data_[i])); // Manual move
                data_[i].~T();
            }
            
            free(data_);
            data_ = new_data;
            capacity_ = new_capacity;
        }
        
    public:
        /** Default constructor. */
        AliceArray() noexcept = default;
        
        /**
         * @brief Construct with n default-initialized elements.
         * @param n Number of elements.
         */
        explicit AliceArray(uint64_t n) noexcept {
            resize(n);
        }
        
        /**
         * @brief Construct with n elements, all set to initialValue.
         * @param n Number of elements.
         * @param initialValue Value to assign.
         */
        AliceArray(uint64_t n, const T& initialValue) noexcept {
            resize(n);
            for (uint64_t i = 0; i < size_; ++i) {
                data_[i] = initialValue;
            }
        }
        
        /**
         * @brief Construct from raw data pointer and size.
         * @param data Pointer to data.
         * @param n Number of elements.
         */
        AliceArray(const T* data, uint64_t n) noexcept {
            if (data && n > 0) {
                reserve(n);
                for (uint64_t i = 0; i < n; ++i) {
                    push_back(data[i]);
                }
            }
        }
        
        /** Copy constructor. */
        AliceArray(const AliceArray& other) noexcept {
            if (other.size_ > 0) {
                reserve(other.size_);
                for (uint64_t i = 0; i < other.size_; ++i) {
                    push_back(other.data_[i]);
                }
            }
        }

        /**
         * @brief Construct from initializer list.
         * @tparam U Type of initializer list elements.
         * @param init Initializer list.
         */
        template<typename U>
        AliceArray(std::initializer_list<U> init) noexcept {
            reserve(init.size());
            for (const auto& v : init) {
                push_back(static_cast<T>(v));
            }
        }
        
        /** Copy assignment. */
        AliceArray& operator=(const AliceArray& other) noexcept {
            if (this != &other) {
                clear();
                if (other.size_ > 0) {
                    reserve(other.size_);
                    for (uint64_t i = 0; i < other.size_; ++i) {
                        push_back(other.data_[i]);
                    }
                }
            }
            return *this;
        }
        
        /** Move constructor. */
        AliceArray(AliceArray&& other) noexcept
            : data_(other.data_), size_(other.size_), capacity_(other.capacity_) {
            other.data_ = nullptr;
            other.size_ = 0;
            other.capacity_ = 0;
        }
        
        /** Move assignment. */
        AliceArray& operator=(AliceArray&& other) noexcept {
            if (this != &other) {
                clear();
                free(data_);
                
                data_ = other.data_;
                size_ = other.size_;
                capacity_ = other.capacity_;
                
                other.data_ = nullptr;
                other.size_ = 0;
                other.capacity_ = 0;
            }
            return *this;
        }
        
        /** Destructor. */
        ~AliceArray() noexcept {
            clear();
            free(data_);
        }
        
        /** @return Number of elements. */
        [[nodiscard]] uint64_t size() const noexcept {
            return size_;
        }
        
        /** @return True if array is empty. */
        [[nodiscard]] bool empty() const noexcept {
            return size_ == 0;
        }
        
        /** @return Allocated capacity. */
        [[nodiscard]] uint64_t capacity() const noexcept {
            return capacity_;
        }
        
        /** @return Pointer to data. */
        T* data() noexcept {
            return data_;
        }
        
        /** @return Pointer to data (const). */
        const T* data() const noexcept {
            return data_;
        }
        
        /** @brief Element access. */
        T& operator[](uint64_t i) noexcept {
            return data_[i];
        }
        /** @brief Element access (const). */
        const T& operator[](uint64_t i) const noexcept {
            return data_[i];
        }
        
        /** @return Iterator to beginning. */
        T* begin() noexcept {
            return data_; 
        }
        /** @return Iterator to end. */
        T* end() noexcept { 
            return data_ + size_; 
        }
        /** @return Const iterator to beginning. */
        const T* begin() const noexcept { 
            return data_; 
        }
        /** @return Const iterator to end. */
        const T* end() const noexcept { 
            return data_ + size_; 
        }
        
        /**
         * @brief Add an element to the end.
         * @param value Value to add.
         */
        void push_back(const T& value) noexcept {
            if (size_ >= capacity_) {
                grow_capacity(size_ + 1);
            }
            if (capacity_ > size_) {
                new (data_ + size_) T(value);
                ++size_;
            }
        }
        /**
         * @brief Add an element to the end (alias for push_back).
         * @param value Value to add.
         */
        void emplace_back(const T& value) noexcept {
            push_back(value);
        }
        /**
         * @brief Resize the array to n elements.
         * @param n New size.
         */
        void resize(uint64_t n) noexcept {
            if (n > size_) {
                reserve(n);
                for (uint64_t i = size_; i < n; ++i) {
                    new (data_ + i) T();
                }
            } else if (n < size_) {
                for (uint64_t i = n; i < size_; ++i) {
                    data_[i].~T();
                }
            }
            size_ = n;
        }
        /**
         * @brief Reserve memory for at least n elements.
         * @param n Number of elements.
         */
        void reserve(uint64_t n) noexcept {
            if (n > capacity_) {
                grow_capacity(n);
            }
        }
        /**
         * @brief Shrink capacity to fit size.
         */
        void shrink_to_fit() noexcept {
            if (size_ < capacity_) {
                if (size_ == 0) {
                    free(data_);
                    data_ = nullptr;
                    capacity_ = 0;
                } else {
                    T* new_data = static_cast<T*>(malloc(size_ * sizeof(T)));
                    if (new_data) {
                        for (uint64_t i = 0; i < size_; ++i) {
                            new (new_data + i) T(static_cast<T&&>(data_[i])); // Manual move
                            data_[i].~T();
                        }
                        free(data_);
                        data_ = new_data;
                        capacity_ = size_;
                    }
                }
            }
        }
        /**
         * @brief Clear the array, destroying all elements.
         */
        void clear() noexcept {
            for (uint64_t i = 0; i < size_; ++i) {
                data_[i].~T();
            }
            size_ = 0;
        }
    };
}