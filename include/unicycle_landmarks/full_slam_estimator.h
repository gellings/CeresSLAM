#pragma once

#include <vector>
#include <Eigen/Core>

#include "ceres/ceres.h"

#include "unicycle_landmarks/estimator_base.h"
#include "unicycle_landmarks/range_bearing_meas.h"
#include "unicycle_landmarks/state.h"

#include "unicycle_landmarks/factors/odometry_factor.h"
#include "unicycle_landmarks/factors/range_bearing_factor.h"
#include "unicycle_landmarks/factors/motion_model_factor.h"
#include "unicycle_landmarks/factors/se2_parameterization.h"
//#include "unicycle_landmarks/factors/angle_local_parameterization.h"

using Eigen::Matrix2d;
using Eigen::Vector2d;
using std::vector;

using ceres::Solve;
using ceres::Solver;

using std::cout;
using std::endl;

class FullSLAMEstimator : public EstimatorBase
{
public:
  FullSLAMEstimator()
  {
    velocities_optimized.setZero();

    velocities_stddev(0) = 0.1;
    velocities_stddev(1) = 0.1;

    states_optimized.push_back(State());
  }

  void printState()
  {
    printf("i:  meas_v meas_w  optimized_v optimized_w x  y theta\n");
    for (int i = 0; i < odometry_meas.size(); i++)
    {
      State state = states_optimized[i];
      printf("%5d: %8.3f %8.3f %8.3f%8.3f%8.3f   %8.3f %8.3f\n", i,
             odometry_meas[i](0), odometry_meas[i](1), velocities_optimized(0),
             velocities_optimized(1), state.x(), state.y(), state.theta());
    }
    printf("Landmarks: \n");
    for (Vector2d lm : landmarks_optimized)
    {
      cout << lm << endl;
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
    states_optimized.push_back(State());
    rb_meas.push_back(z);
  }

  void addParameterBlocks(ceres::Problem& problem)
  {
    ceres::LocalParameterization* se2_local_parameterization =
        SE2LocalParameterization::Create();

    for (int i = 0; i < states_optimized.size(); i++)
    {
      problem.AddParameterBlock(states_optimized[i].arr.data(), 3,
                                se2_local_parameterization);
    }

    // Start from 0
    problem.SetParameterBlockConstant(states_optimized[0].arr.data());
  }

  void addMotionModelFactors(ceres::Problem& problem)
  {
    const double dt = 0.5;
    for (int i = 1; i < states_optimized.size(); i++)
    {
      problem.AddResidualBlock(MotionModelFactor::Create(dt), NULL,
                               states_optimized[i - 1].arr.data(),
                               states_optimized[i].arr.data(),
                               velocities_optimized.data());
    }
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

  void addRangeBearingFactors(ceres::Problem& problem)
  {
    const int num_landmarks = rb_meas[0].ranges.size();
    //landmarks_optimized = vector<Vector2d>(num_landmarks, Vector2d(10., 0.));
    landmarks_optimized = vector<Vector2d>(num_landmarks, Vector2d(-5., 0.));

    for (int i = 0; i < rb_meas.size(); i++)
    {
      for (int lm_idx = 0; lm_idx < num_landmarks; lm_idx++)
      {
        const double range_meas = rb_meas[i].ranges[lm_idx];
        // const double range_var = rb_meas[i].range_variance;
        const double range_var = 0.001;
        //cout << "range: " << range_meas << endl;

        const double bearing_meas = rb_meas[i].bearings[lm_idx];
        // const double bearing_var = rb_meas[i].bearing_variance;
        const double bearing_var = 0.001;
        //cout << "bear: " << bearing_meas << endl;

        // Vector2d curr_lm = landmarks_optimized[lm_idx];
        problem.AddResidualBlock(
            RangeBearingFactor::Create(range_meas, range_var, bearing_meas,
                                       bearing_var),
            NULL, states_optimized[i + 1].arr.data(),
            landmarks_optimized[lm_idx].data());
      }

      if (i == 0)
      {
        //problem.SetParameterBlockConstant(landmarks_optimized[lm_idx].data());
        //velocities_optimized(0) = 0.3;
        //velocities_optimized(1) = 0.6;
        //problem.SetParameterBlockConstant(velocities_optimized.data());
      }
    }
  }

  void solve()
  {
    ceres::Problem problem;

    addParameterBlocks(problem);
    addMotionModelFactors(problem);
    //addOdometryFactors(problem);
    addRangeBearingFactors(problem);

    printState();

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

