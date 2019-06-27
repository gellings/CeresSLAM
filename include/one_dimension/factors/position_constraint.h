#pragma once

#include "ceres/ceres.h"

using ceres::AutoDiffCostFunction;

struct PositionConstraint
{
  typedef AutoDiffCostFunction<PositionConstraint, 1, 1> PositionCostFunction;

  PositionConstraint(double position_meas, double position_stddev)
    : position_meas(position_meas), position_stddev(position_stddev)
  {
  }

  template <typename T>
  bool operator()(const T* const position, T* residual) const
  {
    *residual = (*position - position_meas) / position_stddev;
    return true;
  }

  static PositionCostFunction* Create(const double position_meas,
                                      const double position_stddev)
  {
    return new PositionCostFunction(
        new PositionConstraint(position_meas, position_stddev));
  }

  const double position_meas;
  const double position_stddev;
};
