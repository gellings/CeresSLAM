#pragma once

#include "ceres/ceres.h"

using ceres::AutoDiffCostFunction;

struct LandmarkAnchor
{
  typedef AutoDiffCostFunction<LandmarkAnchor, 1, 1> LandmarkAnchorCostFunc;

  LandmarkAnchor(double prev_value, double stddev)
    : prev_value(prev_value), stddev(stddev)
  {
  }

  template <typename T>
  bool operator()(const T* const curr_value, T* residual) const
  {
    // Simply just penalize how far we deviate from the previous value
    *residual = (*curr_value - prev_value) / stddev;

    return true;
  }

  static LandmarkAnchorCostFunc* Create(const double prev_value,
                                        const double stddev)
  {
    return new LandmarkAnchorCostFunc(new LandmarkAnchor(prev_value, stddev));
  }

  const double prev_value;
  const double stddev;
};
