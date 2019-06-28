#pragma once

#include "ceres/ceres.h"

using ceres::AutoDiffCostFunction;

struct VelocityConstraint
{
  typedef AutoDiffCostFunction<VelocityConstraint, 1, 1, 1, 1>
      VelocityCostFunction;

  VelocityConstraint(double dt, double velocity_stddev)
    : dt(dt), velocity_stddev(velocity_stddev)
  {
  }

  template <typename T>
  bool operator()(const T* const pos1, const T* const pos2, const T* const vel,
                  T* residual) const
  {
    *residual = (((*pos2 - *pos1) / dt) - *vel) / velocity_stddev;
    return true;
  }

  static VelocityCostFunction* Create(const double dt,
                                      const double velocity_stddev)
  {
    return new VelocityCostFunction(
        new VelocityConstraint(dt, velocity_stddev));
  }

  const double dt;
  const double velocity_stddev;
};
