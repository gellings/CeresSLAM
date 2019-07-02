#pragma once

#include <vector>
#include <Eigen/Core>

#include "ceres/ceres.h"

#include "unicycle_landmarks/estimator_base.h"
#include "unicycle_landmarks/range_bearing_meas.h"
#include "unicycle_landmarks/state.h"

#include "unicycle_landmarks/factors/odometry_factor.h"

using Eigen::Matrix2d;
using Eigen::Vector2d;
using std::vector;

using ceres::Solve;
using ceres::Solver;

class FullSLAMEstimator : public EstimatorBase
{
public:
  FullSLAMEstimator()
  {
    velocities_optimized.setZero();

    velocities_stddev(0) = 0.1;
    velocities_stddev(1) = 0.1;
  }

  void printState()
  {
    printf("i:  meas_v meas_w  optimized_v optimized_w\n");
    for (int i = 0; i < odometry_meas.size(); i++)
    {
      printf("%5d: %8.3f %8.3f %8.3f %8.3f\n", i, odometry_meas[i](0),
             odometry_meas[i](1), velocities_optimized(0),
             velocities_optimized(1));
    }
  }

  // t - current time (seconds)
  // z - measurement
  // R - covariance
  void odometryCallback(const double& t, const Vector2d& z, const Matrix2d& R)
  {
    odometry_meas.push_back(z);
  }

  void rangeBearingCallback(const double& t, const RangeBearingMeas& z)
  {
    rb_meas.push_back(z);
    // std::cout << "rbs callback" << std::endl;
    // for (unsigned int i = 0; i < z.ranges.size(); i++)
    //{
    // std::cout << "lm #" << i << " range: " << z.ranges[i]
    //<< " bear: " << z.bearings[i] << std::endl;
    //}
  }

  void addOdometryFactors(ceres::Problem& problem)
  {
    for (Vector2d odom_meas : odometry_meas)
    {
      problem.AddResidualBlock(
          OdometryFactor::Create(odom_meas, velocities_stddev), NULL,
          velocities_optimized.data());
    }
  }

  void solve()
  {
    printState();
    ceres::Problem problem;

    addOdometryFactors(problem);

    Solver::Options solver_options;
    Solver::Summary summary;
    Solve(solver_options, &problem, &summary);

    printState();
  }

  vector<Vector2d> odometry_meas;
  vector<RangeBearingMeas> rb_meas;

  // double velocity_optimized;
  // double omega_optimized;
  Vector2d velocities_optimized;
  Vector2d velocities_stddev;

  vector<State> states_optimized;
  vector<Vector2d> landmarks_optimized;
};

