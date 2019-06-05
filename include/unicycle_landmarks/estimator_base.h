#pragma once

#include <Eigen/Core>

#include "unicycle_landmarks/range_bearing_meas.h"

class EstimatorBase
{
public:
  // t - current time (seconds)
  // z - measurement
  // R - covariance
  virtual void odometryCallback(const double& t, const Eigen::Vector2d& z,
                                const Eigen::Matrix2d& R)
  {
  }

  virtual void rangeBearingCallback(const double& t, const RangeBearingMeas& z)
  {
  }
};
