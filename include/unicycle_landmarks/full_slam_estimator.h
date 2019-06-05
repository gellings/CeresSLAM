#pragma once

#include <Eigen/Core>

#include "unicycle_landmarks/estimator_base.h"
#include "unicycle_landmarks/range_bearing_meas.h"

class FullSLAMEstimator : public EstimatorBase
{
public:
  // t - current time (seconds)
  // z - measurement
  // R - covariance
  virtual void odometryCallback(const double& t, const Eigen::Vector2d& z,
                                const Eigen::Matrix2d& R)
  {
    //std::cout << "odom callback" << std::endl;
    //std::cout << "odom meas: " << std::endl << z << std::endl;
  }

  virtual void rangeBearingCallback(const double& t, const RangeBearingMeas& z)
  {
    //std::cout << "rbs callback" << std::endl;
    //for (unsigned int i = 0; i < z.ranges.size(); i++)
    //{
      //std::cout << "lm #" << i << " range: " << z.ranges[i]
                //<< " bear: " << z.bearings[i] << std::endl;
    //}
  }
};

