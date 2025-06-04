#ifndef CONTAINERS_H
#define CONTAINERS_H

#include <array>
#include <cstddef>
#include <ento-util/debug.h>
#include <optional>
#include <vector>

namespace EntoUtil
{

// EntoArray is backed by a std::array but exposes functions that 
// are the same as used in std::vector. It allows for easy interchangeability
// between the fixed size and dynamic sized containers, where the former are
// used on an MCU, and latter used when running natively on host computer.
template <typename T, std::size_t Capacity>
class EntoArray
{
public:
  constexpr EntoArray() : size_(0) {}

  T* data() { return data_.data(); }
  const T* data() const { return data_.data(); }  

  // Add a new element to the end of the array
  bool push_back(const T& value) {
    if (size_ >= Capacity) {
      ENTO_DEBUG("Pushed item onto EntoArray that is at capacity!");
      return false;
    }
    data_[size_++] = value;
    return true;
  }

  bool push_back(T&& value) {
    if (size_ >= Capacity) {
      ENTO_DEBUG("Pushed item onto EntoArray that is at capacity!");
      return false;
    }
    data_[size_++] = std::move(value);
    return true;
  }

  template <typename... Args>
  void emplace_back(Args&&... args)
  {
    if (size_ >= Capacity)
    {
      ENTO_DEBUG("Emplaced item into EntoArray that is at capacity!");
    }

    // Use placement new to construct the object in-place
    data_[size_] = T(std::forward<Args>(args)...);
    ++size_;
  }

  // Remove the last element from the array
  std::optional<T> pop_back() {
    if (size_ == 0) {
      ENTO_DEBUG("Popped item from EntoArray that is empty!");
      return std::nullopt;
    }
    return data_[--size_];
  }

  // Access the last element
  std::optional<T> back() const {
    if (size_ == 0) {
      ENTO_DEBUG("Accessing back of empty array...");
      return std::nullopt;
    }
    return data_[size_ - 1];
  }

  // Access elements via index
  T& operator[](std::size_t index) {
    return data_[index];
  }

  const T& operator[](std::size_t index) const {
    return data_[index];
  }

  // Get the current size of the array
  constexpr std::size_t size() const {
    return size_;
  }

  // Check if the array is empty
  constexpr bool empty() const {
    return size_ == 0;
  }

  // Check if the array is full
  constexpr bool full() const {
    return size_ == Capacity;
  }

  constexpr std::size_t capacity() const {
    return Capacity;
  }

  // Clear the array
  void clear() {
    size_ = 0;
  }

  T* begin() { return data_.data(); }
  const T* begin() const { return data_.data(); }

  T* end() { return data_.data() + size_; }
  const T* end() const { return data_.data() + size_; }

private:
  std::array<T, Capacity> data_;
  std::size_t size_;
};

template <typename T, std::size_t Capacity>
struct ContainerSelector
{
  using type = std::conditional_t<
      Capacity == 0,       // If Capacity is 0
      std::vector<T>,      // Use std::vector
      EntoArray<T, Capacity>>; // Otherwise, use EntoArray
};

// Alias for convenience
template <typename T, std::size_t Capacity = 0>
using EntoContainer = typename ContainerSelector<T, Capacity>::type;


} // namespace EntoUtil
  //
#endif // CONTAINERS_H
