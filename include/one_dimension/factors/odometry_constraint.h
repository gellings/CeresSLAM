#pragma once

#include "ceres/ceres.h"

using ceres::AutoDiffCostFunction;

struct OdometryConstraint
{
  typedef AutoDiffCostFunction<OdometryConstraint, 1, 1> OdometryCostFunction;
  OdometryConstraint(double odometry_meas, double odometry_stddev)
    : odometry_meas(odometry_meas), odometry_stddev(odometry_stddev)
  {
  }
  template <typename T>
  bool operator()(const T* const solved_odometry, T* residual) const
  {
    *residual = (*solved_odometry - odometry_meas) / odometry_stddev;
    return true;
  }
  static OdometryCostFunction* Create(const double odometry_meas,
                                      const double odometry_stddev)
  {
    return new OdometryCostFunction(
        new OdometryConstraint(odometry_meas, odometry_stddev));
  }
  const double odometry_meas;
  const double odometry_stddev;
};
