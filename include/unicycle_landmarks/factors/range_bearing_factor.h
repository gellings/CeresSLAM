#pragma once

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
    // Compute residual for range
    const T pos_x = state[0];
    const T pos_y = state[1];
    const T lm_x = landmark[0];
    const T lm_y = landmark[1];

    const T diff_x = (lm_x - pos_x);
    const T diff_y = (lm_y - pos_y);
    //const T range_hat = sqrt(diff_x * diff_x + diff_y * diff_y);
    const T range_hat = (diff_x * diff_x + diff_y * diff_y);
    residual[0] =
        //(range_hat - static_cast<T>(range_meas * range_meas)) / static_cast<T>(range_stddev);
      static_cast<T>(range_meas * range_meas) - diff_x * diff_x - diff_y * diff_y;
    // residual[0] = (range_meas - range_hat) / range_stddev;

    // Compute residual for bearing
    // T ang_to_lm;
    // if ((abs(diff_x) > 1e-8) && (abs(diff_y) > 1e-8))
    //{
    // ang_to_lm = atan2(diff_y, diff_x);
    //}
    // else
    //{
    // ang_to_lm = T(0.);
    //}

    const T ang_to_lm = atan2(diff_y, diff_x);
    // const T bearing = NormalizeAngle(ang_to_lm - state[2]);
    const T bearing = (ang_to_lm - state[2]);
    const T normalized_error =
        NormalizeAngle(bearing - static_cast<T>(bearing_meas));
    residual[1] = normalized_error / static_cast<T>(bearing_stddev);

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
