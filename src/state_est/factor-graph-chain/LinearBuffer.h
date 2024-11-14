#ifndef __LINEAR_BUFFER_H__
#define __LINEAR_BUFFER_H__

#include <array>
#include <cassert>

template <typename T, int Size, int Storage>
class LinearBuffer
{
  private:
    std::array< T, Storage > m_buffer;
    int m_head;
    int m_tail;
    int m_count;

  public:
    LinearBuffer() :
        m_head(0),
        m_tail(0),
        m_count(0)
    {}

    void push( const T& value )
    {
        assert( m_head < Storage );
        m_buffer[m_head] = value;
        m_head++;
        if ( m_count < Size )
        {
            m_count++;
        }
        else {
            m_tail++;
        }
    }

    T& operator[]( int index )
    {
        int true_index = ( index + m_tail );
        return m_buffer[true_index];
    }

    int size() const
    {
        return m_count;
    }

    std::array< T, Storage >& get_array()
    {
        return m_buffer;
    }
};

#endif // __LINEAR_BUFFER_H__
