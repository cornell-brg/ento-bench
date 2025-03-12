#ifndef ENTO_ALLOC_H
#define ENTO_ALLOC_H

#include <cstddef>
#include <cstdint>
#include <type_traits>

class ArenaAllocator
{
public:
  using value_type = T;

  ArenaAllocator(uint8_t* arena, std::size_t size)
    : arena_(arena), size_(size), offset_(0) {}

  template <typename T>
  T* allocate(size_t n)
  {
    static_assert(alignof(T) <= alignof(std::max_align_t), "Unsupported alignment.");

    // Ensure current offset respects alignment
    size_t alignment = alignof(T);
    offset_ = align_up(offset_, alignment);
  }
private:
  uint8_t* arena_;
  size_t size_;
  size_t offset_;

  static size_t align_up(size_t offset, size_t alignment)
  {
    return (offset + alignment - 1) & ~(alignment - 1);
  }
};

#endif // ENTO_ALLOC_H
