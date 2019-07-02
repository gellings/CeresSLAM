#pragma once

#include "ceres/ceres.h"

#include <Eigen/Core>

using ceres::AutoDiffCostFunction;
using Eigen::Vector2d;

struct OdometryFactor
{
  typedef AutoDiffCostFunction<OdometryFactor, 2, 2> OdometryCostFunction;
  OdometryFactor(Vector2d odometry_meas, Vector2d odometry_stddev)
    : odometry_meas(odometry_meas), odometry_stddev(odometry_stddev)
  {
  }
  template <typename T>
  bool operator()(const T* const optimized_odom, T* residual) const
  {
    residual[0] = (optimized_odom[0] - odometry_meas(0)) / odometry_stddev(0);
    residual[1] = (optimized_odom[1] - odometry_meas(1)) / odometry_stddev(1);

    return true;
  }
  static OdometryCostFunction* Create(const Vector2d odometry_meas,
                                      const Vector2d odometry_stddev)
  {
    return new OdometryCostFunction(
        new OdometryFactor(odometry_meas, odometry_stddev));
  }
  const Vector2d odometry_meas;
  const Vector2d odometry_stddev;
};
