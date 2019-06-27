#pragma once

#include "ceres/ceres.h"

using ceres::AutoDiffCostFunction;

struct RangeConstraint
{
  typedef AutoDiffCostFunction<RangeConstraint, 1, 1> RangeCostFunction;

  RangeConstraint(double range_meas, double range_stddev)
    : range_meas(range_meas), range_stddev(range_stddev)
  {
  }

  template <typename T>
  bool operator()(const T* const position, T* residual) const
  {
    *residual = (*position - range_meas) / range_stddev;
    return true;
  }

  static RangeCostFunction* Create(const double range_meas,
                                   const double range_stddev)
  {
    return new RangeCostFunction(new RangeConstraint(range_meas, range_stddev));
  }

  const double range_meas;
  const double range_stddev;
};
