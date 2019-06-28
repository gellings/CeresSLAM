#pragma once

#include "one_dimension/estimator_base.h"
#include "one_dimension/range_meas.h"
//#include "one_dimension/factors/odometry_constraint.h"
#include "one_dimension/factors/range_constraint.h"
#include "one_dimension/factors/position_constraint.h"
#include "one_dimension/factors/velocity_constraint.h"

using std::vector;

using ceres::Solver;

class FullSLAMEstimator : public EstimatorBase
{
public:
  FullSLAMEstimator()
  {
    position_meas.push_back(0.);
    position_optimized.push_back(0.);

  }

  // t - current time (seconds)
  // z - measurement
  // R - covariance
  virtual void positionCallback(const double& t, const double& z,
                                const double& R)
  {
    position_stddev_ = sqrt(R);
    //position_meas.push_back(z);
    //position_optimized.push_back(z);
    position_meas.push_back(0.);
    position_optimized.push_back(0.);
  }

  virtual void odometryCallback(const double& t, const double& z,
                                const double& R)
  {
    // odom_R_ = R;
    // odom_values.push_back(z);
  }

  virtual void rangeCallback(const double& t, const RangeMeas& z)
  {
    range_stddev_ = sqrt(z.range_variance);
    range_meas.push_back(z.ranges);

    if (num_landmarks == 0)
    {
      num_landmarks = range_meas[0].size();
      landmarks_optimized = vector<double>(num_landmarks, 0.);
    }
  }

  void solve()
  {
    velocity_optimized = 0.;
    position_optimized = position_meas;
    //position_optimized = vector<double>(position_meas.size(), 0.);

    ceres::Problem problem;

    problem.AddParameterBlock(&(position_optimized[0]), 1);
    problem.SetParameterBlockConstant(&(position_optimized[0]));

    for (int i = 1; i < position_meas.size(); i++)
    {
      // Create and add a cost for the position Contraint for pose i
      //problem.AddResidualBlock(
          //PositionConstraint::Create(position_meas[i], position_stddev_), NULL,
          //&position_optimized[i]);

      // Create a cost for the velocity constraint between pose i, i-1
      problem.AddResidualBlock(VelocityConstraint::Create(velocity_stddev),
                               NULL, &(position_optimized[i - 1]),
                               &(position_optimized[i]), &velocity_optimized);

      // Landmark constraints
      for (int lm_idx = 0; lm_idx < num_landmarks; lm_idx++)
      {
        if (range_meas[i-1][lm_idx] > 0.)
        {
          problem.AddResidualBlock(
              RangeConstraint::Create(range_meas[i-1][lm_idx], range_stddev_),
              NULL, &(position_optimized[i]), &(landmarks_optimized[lm_idx]));
        }
      }
    }

    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = true;

    Solver::Summary summary;
    printf("Solving...\n");
    Solve(solver_options, &problem, &summary);
    printf("Done.\n");
    // std::cout << summary.FullReport() << "\n";
    std::cout << "Final odom: " << velocity_optimized << std::endl;
  }

  double velocity_optimized;
  double velocity_stddev = 0.001;

  double position_stddev_;
  vector<double> position_meas;
  vector<double> position_optimized;

  int num_landmarks = 0;
  vector<double> landmarks_optimized;

  // double odom_R_;
  // vector<double> odom_values;

  double range_stddev_;
  vector<vector<double>> range_meas;
};

