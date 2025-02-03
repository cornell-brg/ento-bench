#ifndef AXLE_UTIL_H
#define AXLE_UTIL_H

#include <cmath>
#include "CirculeBuffer.h"

inline double mod2pi(double angle)
{
  return fmod(angle + 2 * M_PI, 2 * M_PI);
}

inline float mod2pi(float angle)
{
  return fmodf(angle + 2 * M_PI, 2 * M_PI);
}

inline float cosine(float x)
{
  return cosf(x);
}

inline double cosine(double x)
{
  return cos(x);
}

inline float sine(float y)
{
  return sinf(y);
}

inline double sine(double y)
{
  return sin(y);
}


template <typename Scalar, size_t WindowSize>
class UnicycleSlidingWindow
{
public: 
  void push_state(const Eigen::Matrix<Scalar, 4, 1>& state)
  {
    states.push(state);
  }

  void push_kinematics_jacobian(const Eigen::Matrix<Scalar, 4, 4>& jacobian)
  {
    kinematicsJacobians.push(jacobian);
  }

  void push_kinematics_jacobian(const Eigen::Matrix<Scalar, 2, 4>& jacobian)
  {
    observationJacobians.push(jacobian);
  }

  void push_Ai(const Eigen::Matrix<Scalar, 4, 4>& Ai)
  {
    Ais.push(Ai);
  }

  void push_Ei(const Eigen::Matrix<Scalar, 4, 1>& Ei)
  {
    Eis.push(Ei);
  }

private:
  CircularBuffer<Eigen::Matrix<Scalar, 4, 1>, WindowSize> states;
  CircularBuffer<Eigen::Matrix<Scalar, 4, 4>, WindowSize> kinematicsJacobians;
  CircularBuffer<Eigen::Matrix<Scalar, 2, 4>, WindowSize> observationJacobians;
  CircularBuffer<Eigen::Matrix<Scalar, 4, 4>, WindowSize> Ais;
  CircularBuffer<Eigen::Matrix<Scalar, 4, 1>, WindowSize> Eis;
}

#endif // AXLE_UTIL_H
