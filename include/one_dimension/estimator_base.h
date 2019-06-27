#pragma once

#include "one_dimension/range_meas.h"

class EstimatorBase
{
public:
  // t - current time (seconds)
  // z - measurement
  // R - covariance
  virtual void odometryCallback(const double& t, const double& z,
                                const double& R)
  {
  }

  virtual void rangeCallback(const double& t, const RangeMeas& z)
  {
  }
};
