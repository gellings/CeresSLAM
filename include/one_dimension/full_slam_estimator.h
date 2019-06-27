#pragma once

#include "one_dimension/estimator_base.h"
#include "one_dimension/range_meas.h"

class FullSLAMEstimator : public EstimatorBase
{
public:
  // t - current time (seconds)
  // z - measurement
  // R - covariance
  virtual void odometryCallback(const double& t, const double& z,
                                const double& R)
  {
    //std::cout << "odom callback" << std::endl;
    //std::cout << "odom meas: " << std::endl << z << std::endl;
  }

  virtual void rangeCallback(const double& t, const RangeMeas& z)
  {
    //std::cout << "rbs callback" << std::endl;
    //for (unsigned int i = 0; i < z.ranges.size(); i++)
    //{
      //std::cout << "lm #" << i << " range: " << z.ranges[i]
                //<< " bear: " << z.bearings[i] << std::endl;
    //}
  }
};

