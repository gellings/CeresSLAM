#pragma once

#include "one_dimension/estimator_base.h"
#include "one_dimension/range_meas.h"
#include "one_dimension/factors/range_constraint.h"
#include "one_dimension/factors/position_constraint.h"
#include "one_dimension/factors/velocity_constraint.h"
#include "one_dimension/factors/landmark_anchor.h"

#include <utility>
#include "utils/circular_buffer.h"

using std::vector;

using ceres::Solver;

class OnlineSLAMEstimator : public EstimatorBase
{
public:
  OnlineSLAMEstimator() : optimized_time_pos(10), range_meas(9)
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
    //range_meas.push_back(z.ranges);
    range_meas.put(z.ranges);

    //auto last_pos = optimized_time_pos.get_newest();
    //const double dt = t - last_pos.first;
    //const double predicted_x = last_pos.second + velocity_optimized * dt;

    const double last_pos_x = optimized_time_pos.get_newest().second;
    const double last_pos_t = optimized_time_pos.get_newest().first;
    const double dt = t - last_pos_t;
    const double predicted_x = last_pos_x + velocity_optimized * dt;
    //std::cout << "BEFORE" << std::endl;
    //for (int i = 0; i < optimized_time_pos.size(); i++)
    //{
      //std::cout << optimized_time_pos[i].second << std::endl;
    //}
    //std::cout << "AFTER" << std::endl;
    optimized_time_pos.put(std::make_pair(t, predicted_x));
    //for (int i = 0; i < optimized_time_pos.size(); i++)
    //{
      //std::cout << optimized_time_pos[i].second << std::endl;
    //}
    //std::cout << "last pos: " << optimized_time_pos.get_newest().second << std::endl;
    //std::cout << "put: " << predicted_x << std::endl;
    //static double putter = 0.;
    //double putter = last_pos.second + 1.;
    //optimized_time_pos.put(std::make_pair(t, putter));
    //std::cout << "last pos: " << optimized_time_pos.get_newest().second << std::endl;
    //std::cout << "just put: " << putter << std::endl;

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
    //problem.AddParameterBlock(&(optimized_time_pos[0].second), 1);
    //problem.SetParameterBlockConstant(&(optimized_time_pos[0].second));

    printf("\n\nNEXT ITER\n");
    std::cout << "pos size: " << optimized_time_pos.size() << std::endl;
    std::cout << "ranges size: " << range_meas.size() << std::endl;
    //for (int i = 0; i < optimized_time_pos.size(); i++)
    //{
      //std::cout << optimized_time_pos[i].second << std::endl;
    //}

    for (int i = 1; i < optimized_time_pos.size(); i++)
    {
      // Create a cost for the velocity constraint between pose i, i-1
      const double dt =
          optimized_time_pos[i].first - optimized_time_pos[i - 1].first;
      //std::cout << "i-1: " << optimized_time_pos[i -1].second << " i: " << optimized_time_pos[i].second << std::endl;
      problem.AddResidualBlock(VelocityConstraint::Create(dt, velocity_stddev),
                               NULL, &(optimized_time_pos[i - 1].second),
                               &(optimized_time_pos[i].second),
                               &velocity_optimized);
      if (i == 1)
      {
        problem.AddResidualBlock(
            LandmarkAnchor::Create(optimized_time_pos[0].second, 0.001), NULL,
            &(optimized_time_pos[0].second));
      }

      // Landmark constraints
      for (int lm_idx = 0; lm_idx < num_landmarks; lm_idx++)
      {
        if (i == 9)
        {
           //Once per landmark make an anchor
          problem.AddResidualBlock(
              LandmarkAnchor::Create(landmarks_optimized[lm_idx], 0.1), NULL,
              &(landmarks_optimized[lm_idx]));
          //std::cout << "added lm: " << lm_idx << " pos: " << landmarks_optimized[lm_idx] << std::endl;

          //problem.AddParameterBlock(&(landmarks_optimized[lm_idx]), 1);
          //problem.SetParameterBlockConstant(&(landmarks_optimized[lm_idx]));
        }

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
    //std::cout << summary.FullReport() << "\n";
    // std::cout << "Optimized Velocity : " << velocity_optimized << std::endl;
    // printState();
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
  //vector<vector<double>> range_meas;
  CircularBuffer<vector<double>> range_meas;
};

