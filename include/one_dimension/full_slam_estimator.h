#pragma once

#include "one_dimension/estimator_base.h"
#include "one_dimension/range_meas.h"
//#include "one_dimension/factors/odometry_constraint.h"
#include "one_dimension/factors/range_constraint.h"
#include "one_dimension/factors/velocity_constraint.h"

using std::vector;

using ceres::Solver;

class FullSLAMEstimator : public EstimatorBase
{
public:
  // t - current time (seconds)
  // z - measurement
  // R - covariance
  virtual void odometryCallback(const double& t, const double& z,
                                const double& R)
  {
    // std::cout << "odom callback" << std::endl;
    // std::cout << "odom meas: " << std::endl << z << std::endl;
    odom_R_ = R;
    odom_values.push_back(z);
  }

  virtual void rangeCallback(const double& t, const RangeMeas& z)
  {
    // std::cout << "rbs callback" << std::endl;
    // for (unsigned int i = 0; i < z.ranges.size(); i++)
    //{
    // std::cout << "lm #" << i << " range: " << z.ranges[i]
    //<< " bear: " << z.bearings[i] << std::endl;
    //}
    range_stddev_ = sqrt(z.range_variance);
    const double range_value = z.ranges[0];
    range_readings.push_back(range_value);
  }

  //// This way works to average all odoms
  // void solve()
  //{
  // final_odom = -999.;

  // ceres::Problem problem;

  // for (int i = 0; i < odom_values.size(); i++)
  //{
  //// Create and add an AutoDiffCostFunction for the Odometry Contraint for
  //// pose i
  // problem.AddResidualBlock(
  // OdometryConstraint::Create(odom_values[i], odom_R_), NULL,
  //&final_odom);
  //}

  // ceres::Solver::Options solver_options;
  // solver_options.minimizer_progress_to_stdout = true;

  // Solver::Summary summary;
  // printf("Solving...\n");
  // Solve(solver_options, &problem, &summary);
  // printf("Done.\n");
  // std::cout << summary.FullReport() << "\n";
  // std::cout << "Final odom: " << final_odom << std::endl;
  //}

  void solve()
  {
    final_odom = 2.;
    //std::fill(range_solved.zeros(2);
    range_solved = vector<double>(range_readings.size(), 0);

    ceres::Problem problem;

    for (int i = 0; i < odom_values.size(); i++)
    {
      // Create and add an AutoDiffCostFunction for the Range Contraint for
      // pose i
      problem.AddResidualBlock(
          RangeConstraint::Create(range_readings[i], range_stddev_), NULL,
          &range_solved[i]);

      if (i > 0)
      {
        // Create a cost for the velocity constraint between pose i, i-1
        problem.AddResidualBlock(VelocityConstraint::Create(velocity_stddev),
                                 NULL, &(range_solved[i - 1]),
                                 &(range_solved[i]), &final_odom);
      }
    }

    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = true;

    Solver::Summary summary;
    printf("Solving...\n");
    Solve(solver_options, &problem, &summary);
    printf("Done.\n");
    //std::cout << summary.FullReport() << "\n";
    std::cout << "Final odom: " << final_odom << std::endl;
  }

  double final_odom;
  double velocity_stddev = 0.001;

  double odom_R_;
  vector<double> odom_values;

  double range_stddev_;
  vector<double> range_readings;
  vector<double> range_solved;
};

