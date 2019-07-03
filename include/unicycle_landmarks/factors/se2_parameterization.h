#pragma once
#include "ceres/local_parameterization.h"
#include "normalize_angle.h"

// Defines a local parameterization for updating the angle to be constrained in
// [-pi to pi).
class SE2LocalParameterization
{
public:
  template <typename T>
  bool operator()(const T* state, const T* delta_state,
                  T* new_state) const
  {
    new_state[0] = state[0] + delta_state[0];
    new_state[1] = state[1] + delta_state[1];
    new_state[2] =
        NormalizeAngle(state[2] + delta_state[2]);
    return true;
  }
  static ceres::LocalParameterization* Create()
  {
    return (new ceres::AutoDiffLocalParameterization<SE2LocalParameterization,
                                                     3, 3>);
  }
};
