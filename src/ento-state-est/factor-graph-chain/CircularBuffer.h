#ifndef _CIRCULAR_BUFFER_H__
#define _CIRCULAR_BUFFER_H__

#include <array>

template <typename T, int Size>
class CircularBuffer
{
  private:
    std::array<T, Size> m_buffer;
    int m_head;
    int m_tail;
    int m_count;

  public:
    CircularBuffer() :
        m_head(0),
        m_tail(0),
        m_count(0)
    {}

    void push( const T& value )
    {
        m_buffer[m_head] = value;
        m_head = ( m_head + 1 ) % Size;
        if ( m_count < Size )
        {
            m_count++;
        }
        else {
            m_tail = ( m_tail + 1 ) % Size;
        }
    }

    T& operator[]( int index )
    {
        int true_index = ( index + m_tail ) % Size;
        return m_buffer[true_index];
    }

    int size() const
    {
        return m_count;
    }
};

#endif // __CIRCULAR_BUFFER_H__
