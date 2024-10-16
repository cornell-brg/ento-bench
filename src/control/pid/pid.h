#ifndef __PID_H__
#define __PID_H__

template< typename T >
class PIDController
{
  private:
    T m_integral;
    T m_setpoint;
    T m_perror;
    T m_p;
    T m_i;
    T m_d;
  public:
    PIDController(){}

    void reset( T new_setpoint )
    {
      m_integral = 0;
      m_setpoint = new_setpoint;
      m_perror = 0;
    }

    void set_weights( T p, T i, T d )
    {
      m_p = p;
      m_i = i;
      m_d = d;
    }

    T update( T curr_val )
    {
      T new_error = m_setpoint - curr_val;
      T p_contrib = new_error * m_p;
      T d_contrib = ( new_error - m_perror ) * m_d;
      m_perror = new_error;
      T i_contrib = m_integral + ( new_error * m_i );
      m_integral = i_contrib;
      return ( p_contrib + i_contrib - d_contrib );
    }
};

#endif // __PID_H__
