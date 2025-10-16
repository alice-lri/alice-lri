#pragma once
#include <cstdint>
#include <cstdlib>
#include <initializer_list>

namespace alice_lri {
    template<class T>
    class AliceArray {
    private:
        T* data_ = nullptr;
        uint64_t size_ = 0;
        uint64_t capacity_ = 0;
        
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
        AliceArray() noexcept = default;
        
        explicit AliceArray(uint64_t n) noexcept {
            resize(n);
        }
        
        AliceArray(uint64_t n, const T& initialValue) noexcept {
            resize(n);
            for (uint64_t i = 0; i < size_; ++i) {
                data_[i] = initialValue;
            }
        }
        
        AliceArray(const T* data, uint64_t n) noexcept {
            if (data && n > 0) {
                reserve(n);
                for (uint64_t i = 0; i < n; ++i) {
                    push_back(data[i]);
                }
            }
        }
        
        AliceArray(const AliceArray& other) noexcept {
            if (other.size_ > 0) {
                reserve(other.size_);
                for (uint64_t i = 0; i < other.size_; ++i) {
                    push_back(other.data_[i]);
                }
            }
        }

        template<typename U>
        AliceArray(std::initializer_list<U> init) noexcept {
            reserve(init.size());
            for (const auto& v : init) {
                push_back(static_cast<T>(v));
            }
        }
        
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
        
        AliceArray(AliceArray&& other) noexcept
            : data_(other.data_), size_(other.size_), capacity_(other.capacity_) {
            other.data_ = nullptr;
            other.size_ = 0;
            other.capacity_ = 0;
        }
        
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
        
        ~AliceArray() noexcept {
            clear();
            free(data_);
        }
        
        [[nodiscard]] uint64_t size() const noexcept {
            return size_;
        }
        
        [[nodiscard]] bool empty() const noexcept {
            return size_ == 0;
        }
        
        [[nodiscard]] uint64_t capacity() const noexcept {
            return capacity_;
        }
        
        T* data() noexcept {
            return data_;
        }
        
        const T* data() const noexcept {
            return data_;
        }
        
        T& operator[](uint64_t i) noexcept {
            return data_[i];
        }
        
        const T& operator[](uint64_t i) const noexcept {
            return data_[i];
        }
        
        T* begin() noexcept {
            return data_; 
        }
        
        T* end() noexcept { 
            return data_ + size_; 
        }
        
        const T* begin() const noexcept { 
            return data_; 
        }
        
        const T* end() const noexcept { 
            return data_ + size_; 
        }
        
        void push_back(const T& value) noexcept {
            if (size_ >= capacity_) {
                grow_capacity(size_ + 1);
            }
            if (capacity_ > size_) {
                new (data_ + size_) T(value);
                ++size_;
            }
        }
        
        void emplace_back(const T& value) noexcept {
            push_back(value);
        }
        
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
        
        void reserve(uint64_t n) noexcept {
            if (n > capacity_) {
                grow_capacity(n);
            }
        }
        
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
        
        void clear() noexcept {
            for (uint64_t i = 0; i < size_; ++i) {
                data_[i].~T();
            }
            size_ = 0;
        }
    };
}