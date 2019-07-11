#pragma once

#include <Eigen/Core>
#include "ceres/ceres.h"

#include "unicycle_landmarks/factors/normalize_angle.h"

using ceres::AutoDiffCostFunction;

struct RangeBearingFactor
{
  typedef AutoDiffCostFunction<RangeBearingFactor, 2, 3, 2>
      RangeBearingCostFunction;

  RangeBearingFactor(double range_meas, double range_stddev,
                     double bearing_meas, double bearing_stddev)
    : range_meas(range_meas)
    , range_stddev(range_stddev)
    , bearing_meas(bearing_meas)
    , bearing_stddev(bearing_stddev)
  {
  }

  template <typename T>
  bool operator()(const T* const state, const T* const landmark,
                  T* residual) const
  {
    // key is to use range and bearing to create mock measurement of landmark in
    // robot frame. If try to use state and landmark variables to create
    // expected range and bearing, you have to use sqrt and atan which the
    // optimizer does not do well with
    const Eigen::Matrix<T, 2, 1> p_robot_world_frame(state);
    const T robot_theta = state[2];

    const Eigen::Matrix<T, 2, 1> landmark_world_frame(landmark);

    Eigen::Map<Eigen::Matrix<T, 2, 1> > residuals_map(residual);
    const Eigen::Matrix<T, 2, 2> R_world_robot = RotationMatrix2D(robot_theta);

    const T meas_landmark_robot_x = T(cos(bearing_meas) * range_meas);
    const T meas_landmark_robot_y = T(sin(bearing_meas) * range_meas);
    const Eigen::Matrix<T, 2, 1> meas_landmark_robot_frame(
        meas_landmark_robot_x, meas_landmark_robot_y);

    const Eigen::Matrix<T, 2, 1> meas_landmark_world_frame =
        R_world_robot.transpose() * meas_landmark_robot_frame +
        p_robot_world_frame;

    residuals_map = meas_landmark_world_frame - landmark_world_frame;

    // Scale by sqrt information matrix
    residuals_map(0) /= 0.00001;
    residuals_map(1) /= 0.00001;

    return true;
  }

  static RangeBearingCostFunction* Create(const double range_meas,
                                          const double range_stddev,
                                          const double bearing_meas,
                                          const double bearing_stddev)
  {
    return new RangeBearingCostFunction(new RangeBearingFactor(
        range_meas, range_stddev, bearing_meas, bearing_stddev));
  }

  const double range_meas;
  const double range_stddev;
  const double bearing_meas;
  const double bearing_stddev;
};
