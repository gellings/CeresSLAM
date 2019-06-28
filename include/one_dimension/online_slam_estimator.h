#pragma once

#include "one_dimension/estimator_base.h"
#include "one_dimension/range_meas.h"
#include "one_dimension/factors/range_constraint.h"
#include "one_dimension/factors/position_constraint.h"
#include "one_dimension/factors/velocity_constraint.h"

#include <utility>
#include "utils/circular_buffer.h"

using std::vector;

using ceres::Solver;

class OnlineSLAMEstimator : public EstimatorBase
{
public:
  OnlineSLAMEstimator() : optimized_time_pos(25)
  {
    velocity_optimized = 0.;
    optimized_time_pos.put(std::make_pair(0., 0.));
  }

  void printState()
  {
    const double true_vel = 3.;
    printf("time:   position    optimized   err\n");
    for (int i = 0; i < optimized_time_pos.size(); i++)
    {
      const double time = optimized_time_pos[i].first;
      const double true_pos = true_vel * time;
      const double est_pos = optimized_time_pos[i].second;
      const double est_err = abs(est_pos - true_pos);
      printf("%8.3f: %8.3f %8.3f %8.3f\n", time, true_pos, est_pos, est_err);
    }
  }

  // t - current time (seconds)
  // z - measurement
  // R - covariance
  virtual void positionCallback(const double& t, const double& z,
                                const double& R)
  {
  }

  virtual void odometryCallback(const double& t, const double& z,
                                const double& R)
  {
  }

  virtual void rangeCallback(const double& t, const RangeMeas& z)
  {
    range_stddev_ = sqrt(z.range_variance);
    range_meas.push_back(z.ranges);

    auto last_pos = optimized_time_pos.get_newest();
    const double dt = t - last_pos.first;
    const double predicted_x = last_pos.second + velocity_optimized * dt;
    optimized_time_pos.put(std::make_pair(t, predicted_x));

    if (num_landmarks == 0)
    {
      num_landmarks = range_meas[0].size();
      landmarks_optimized = vector<double>(num_landmarks, 0.);
    }

    solve();
  }

  void solve()
  {
    ceres::Problem problem;

    // TODO instead only lock down 0. at the beginning
    // TODO make an anchor for x states and landmarks, so it can't differ very
    // much from optimization to optimization. This may be the same with the
    // velocity
    problem.AddParameterBlock(&(optimized_time_pos[0].second), 1);
    problem.SetParameterBlockConstant(&(optimized_time_pos[0].second));

    for (int i = 1; i < optimized_time_pos.size(); i++)
    {
      // Create a cost for the velocity constraint between pose i, i-1
      const double dt =
          optimized_time_pos[i].first - optimized_time_pos[i - 1].first;
      problem.AddResidualBlock(VelocityConstraint::Create(dt, velocity_stddev),
                               NULL, &(optimized_time_pos[i - 1].second),
                               &(optimized_time_pos[i].second),
                               &velocity_optimized);

      // Landmark constraints
      for (int lm_idx = 0; lm_idx < num_landmarks; lm_idx++)
      {
        if (range_meas[i - 1][lm_idx] > 0.)
        {
          problem.AddResidualBlock(
              RangeConstraint::Create(range_meas[i - 1][lm_idx], range_stddev_),
              NULL, &(optimized_time_pos[i].second),
              &(landmarks_optimized[lm_idx]));
        }
      }
    }

    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = false;

    Solver::Summary summary;
    // printf("Solving...\n");
    Solve(solver_options, &problem, &summary);
    // printf("Done.\n");
    // std::cout << summary.FullReport() << "\n";
    // std::cout << "Optimized Velocity : " << velocity_optimized << std::endl;
    //printState();
  }

  double getEstimatedX()
  {
    return optimized_time_pos.get_newest().second;
  }

  double getEstimatedV()
  {
    return velocity_optimized;
  }

  vector<double> getEstimatedLandmarks()
  {
    return landmarks_optimized;
  }

  double velocity_optimized;
  double velocity_stddev = 0.001;

  CircularBuffer<std::pair<double, double>> optimized_time_pos;

  int num_landmarks = 0;
  vector<double> landmarks_optimized;

  double range_stddev_;
  vector<vector<double>> range_meas;
};

