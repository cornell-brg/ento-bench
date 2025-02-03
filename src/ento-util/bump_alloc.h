#include <cstdint>
#include <cstdlib>
#include <cassert>

class BumpAllocator {
public:
  BumpAllocator(uint8_t* buffer, size_t capacity)
      : buffer_(buffer), capacity_(capacity), offset_(0) {}

  void* allocate(size_t size, size_t alignment = 4) {
    uintptr_t current = reinterpret_cast<uintptr_t>(buffer_) + offset_;

    // Align the memory
    uintptr_t aligned = (current + (alignment - 1)) & ~(alignment - 1);

    // Check for overflow
    if (aligned + size > reinterpret_cast<uintptr_t>(buffer_) + capacity_) {
      return nullptr; // Out of memory
    }

    offset_ = static_cast<size_t>(aligned - reinterpret_cast<uintptr_t>(buffer_)) + size;

    return reinterpret_cast<void*>(aligned);
  }

  void reset() {
    offset_ = 0;
  }

  size_t remaining_capacity() const {
    return capacity_ - offset_;
  }

private:
  uint8_t* buffer_;
  size_t capacity_;
  size_t offset_;
};
