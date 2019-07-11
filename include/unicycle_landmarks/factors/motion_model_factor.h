#pragma once

#include "ceres/ceres.h"

#include "unicycle_landmarks/factors/normalize_angle.h"

using ceres::AutoDiffCostFunction;

// Rotation from world to body
template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians)
{
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);

  Eigen::Matrix<T, 2, 2> rotation;
  rotation << cos_yaw, sin_yaw, -sin_yaw, cos_yaw;
  return rotation;
}

struct MotionModelFactor
{
  typedef AutoDiffCostFunction<MotionModelFactor, 3, 3, 3, 2>
      MotionModelCostFunction;

  MotionModelFactor(double dt) : dt(dt)
  {
  }

  template <typename T>
  bool operator()(const T* const state_tm1, const T* const state_t,
                  const T* const velocities, T* residual) const
  {
    const Eigen::Matrix<T, 2, 1> p_a(state_tm1);
    const Eigen::Matrix<T, 2, 1> p_b(state_t);
    const Eigen::Matrix<T, 2, 1> p_diff_world = p_b - p_a;

    const T theta_a = state_tm1[2];
    const Eigen::Matrix<T, 2, 2> R_world_a = RotationMatrix2D(state_tm1[2]);

    const Eigen::Matrix<T, 2, 1> p_diff_a = R_world_a * p_diff_world;

    residual[0] = (((p_diff_a(0)) / dt) - velocities[0]) / 0.00001;
    residual[1] = (((p_diff_a(1)) / dt)) / 0.00001;

    const T theta_b = state_t[2];
    const T omega_hat = NormalizeAngle(theta_b - theta_a) / dt;
    const T omega = velocities[1];
    residual[2] = (omega_hat - omega) / 0.00001;

    return true;
  }

  static MotionModelCostFunction* Create(const double dt)
  {
    return new MotionModelCostFunction(new MotionModelFactor(dt));
  }

  const double dt;
};
