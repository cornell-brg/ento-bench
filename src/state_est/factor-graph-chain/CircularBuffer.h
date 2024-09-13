#ifndef CIRCULAR_BUFFER_HH
#define CIRCULAR_BUFFER_HH

#include <array>

template <typename T, size_t Size>
class CircularBuffer
{
public:
  CircularBuffer() : head_(0), count_(0) {}

  void push(const T& value)
  {
    buffer_[head_] = value;
    head_ = (head_ + 1) % Size;
    if (count_ < Size)
    {
      ++count_;
    }
  }

  const T& operator[](size_t index) const
  {
    size_t true_index = (index + head_) % Size;
    return buffer_[true_index];
  }

  T& front() const
  {
    return buffer_[head_];
  }

  size_t size() const
  {
    return count_;
  }

private:
  std::array<T, Size> buffer_;
  size_t head_;
  size_t count_;
}


#endif // CIRCULAR_BUFFER_HH
