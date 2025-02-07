#ifndef ATTITUDE_MEASUREMENTS_H
#define ATTITUDE_MEASUREMENTS_H

#include <ento-math/core.h>

namespace EntoAttitude
{

//------------------------------------------------------------------------------
// Templated measurement container, specialized on whether magnetometer data is used.
//------------------------------------------------------------------------------
template <typename Scalar, bool UseMag>
struct AttitudeMeasurement;

//------------------------------------------------------------------------------
// Specialization for IMU-only measurements (no magnetometer)
//------------------------------------------------------------------------------
template <typename Scalar>
struct AttitudeMeasurement<Scalar, false>
{
  EntoMath::Vec3<Scalar> gyr;
  EntoMath::Vec3<Scalar> acc;
};

//------------------------------------------------------------------------------
// Specialization for MARG measurements (gyroscope, accelerometer, magnetometer)
//------------------------------------------------------------------------------
template <typename Scalar>
struct AttitudeMeasurement<Scalar, true>
{
  EntoMath::Vec3<Scalar> gyr;
  EntoMath::Vec3<Scalar> acc;
  EntoMath::Vec3<Scalar> mag;
};


//------------------------------------------------------------------------------
// Type aliases for convenience. 
//------------------------------------------------------------------------------
template <typename Scalar>
using IMUMeasurement  = AttitudeMeasurement<Scalar, false>;

template <typename Scalar>
using MARGMeasurement = AttitudeMeasurement<Scalar, true>;



} // namespace EntoAttitude

#endif // ATTITUDE_MEASUREMENTS_H
