#include <math.h>
#include <Eigen/Core>

namespace frame_helper
{

// rotation from vehicle-2 to body frame
inline Eigen::Matrix3d R_v2_to_b(double phi)
{
  Eigen::Matrix3d R_v22b;
  R_v22b << 1,         0,        0,
      0,  cos(phi), sin(phi),
      0, -sin(phi), cos(phi);
  return R_v22b;
}


// rotation from vehicle-1 to vehicle-2 frame
inline Eigen::Matrix3d R_v1_to_v2(double theta)
{
  Eigen::Matrix3d R_v12v2;
  R_v12v2 << cos(theta), 0, -sin(theta),
      0, 1,           0,
      sin(theta), 0,  cos(theta);
  return R_v12v2;
}

// rotation from vehicle to vehicle-1 frame
inline Eigen::Matrix3d R_v_to_v1(double psi)
{
  Eigen::Matrix3d R_v2v1;
  R_v2v1 <<  cos(psi), sin(psi), 0,
      -sin(psi), cos(psi), 0,
      0,        0, 1;
  return R_v2v1;
}

// rotation from vehicle to body frame (3-2-1 Euler)
inline Eigen::Matrix3d R_v_to_b(double phi, double theta, double psi)
{
  return R_v2_to_b(phi) * R_v1_to_v2(theta) * R_v_to_v1(psi);
}
}

