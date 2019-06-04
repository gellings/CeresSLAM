#pragma once

#include <Eigen/Core>

struct State
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum
  {
    X = 0,
    Y = 1,
    THETA = 2,
    V = 3,
    OMEGA = 4,
    SIZE = 5
  };

  Eigen::Matrix<double, SIZE, 1> arr;

  const double& x() const { return arr(X); }
  double& x() { return arr(X); }
  const double& y() const { return arr(Y); }
  double& y() { return arr(Y); }
  const double& theta() const { return arr(THETA); }
  double& theta() { return arr(THETA); }
  const double& v() const { return arr(V); }
  double& v() { return arr(V); }
  const double& omega() const { return arr(OMEGA); }
  double& omega() { return arr(OMEGA); }

};
