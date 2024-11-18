#ifndef CONTAINERS_H
#define CONTAINERS_H

#include <array>
#include <cstddef>
#include <ento-util/debug.h>
#include <optional>

namespace EntoUtil
{

template <typename T, std::size_t Capacity>
class EntoArray {
public:
  constexpr EntoArray() : size_(0) {}

  // Add a new element to the end of the array
  [[nodiscard]] bool push_back(const T& value) {
    if (size_ >= Capacity) {
      ENTO_DEBUG("Pushed item onto EntoArray that is at capacity!");
      return false;
    }
    data_[size_++] = value;
    return true;
  }

  [[nodiscard]] bool push_back(T&& value) {
    if (size_ >= Capacity) {
      ENTO_DEBUG("Pushed item onto EntoArray that is at capacity!");
      return false;
    }
    data_[size_++] = std::move(value);
    return true;
  }

  // Remove the last element from the array
  [[nodiscard]] std::optional<T> pop_back() {
    if (size_ == 0) {
      ENTO_DEBUG("Popped item from EntoArray that is empty!");
      return std::nullopt;
    }
    return data_[--size_];
  }

  // Access the last element
  [[nodiscard]] std::optional<T> back() const {
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
  [[nodiscard]] constexpr bool empty() const {
    return size_ == 0;
  }

  // Check if the array is full
  [[nodiscard]] constexpr bool full() const {
    return size_ == Capacity;
  }

  constexpr std::size_t capacity() const {
    return Capacity;
  }

  // Clear the array
  void clear() {
    size_ = 0;
  }

private:
  std::array<T, Capacity> data_;
  std::size_t size_;
};

} // namespace EntoUtil
  //
#endif // CONTAINERS_H
