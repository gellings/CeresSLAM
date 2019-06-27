#pragma once

#include <Eigen/Core>

struct State
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum
  {
    X = 0,
    V = 1,
    SIZE = 2
  };

  Eigen::Matrix<double, SIZE, 1> arr;

  const double& x() const { return arr(X); }
  double& x() { return arr(X); }
  const double& v() const { return arr(V); }
  double& v() { return arr(V); }

};
