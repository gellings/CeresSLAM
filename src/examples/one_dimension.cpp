#include <iostream>
#include <string>

#include "one_dimension/full_slam_estimator.h"
#include "one_dimension/one_dimension_sim.h"

#include "utils/logger.h"
#include "utils/frame_helper.h"

using std::vector;

void printState(//const vector<double>& position_meas,
                const vector<double>& position_optimized,
                const vector<double>& landmarks_optimized,
                const vector<double>& landmarks_true)
{
  const double true_dt = 0.04;
  const double true_vel = 3.;
  double robot_location = 0.;
  //printf("pose: position    meas     optimized\n");
  printf("pose: position    optimized\n");
  for (int i = 0; i < position_optimized.size(); i++)
  {
    double optimized_err = abs(robot_location - position_optimized[i]);

    printf("%4d: %8.3f %8.3f %8.3f\n", static_cast<int>(i),
           robot_location, position_optimized[i],
           optimized_err);
    robot_location += true_vel * true_dt;
  }
  printf("LMs:  true    optimized     err\n");
  for (int i = 0; i < landmarks_optimized.size(); i++)
  {
    double lm_err = abs(landmarks_true[i] - landmarks_optimized[i]);
        printf("%4d: %8.3f %8.3f %8.3f\n", static_cast<int>(i), landmarks_true[i],
               landmarks_optimized[i], lm_err);
  }
}

int main()
{
  std::string sim_params_yaml_file = "../params/one_dimension/sim_params.yaml";
  bool show_progress_bar = true;
  OneDimensionSimulator sim(sim_params_yaml_file, show_progress_bar);
  Logger log("/tmp/one_dim_sim.bin");

  // Estimator
  FullSLAMEstimator est;
  sim.registerEstimator(&est);

  // Log landmark info
  // log.log(static_cast<double>(sim.num_landmarks_));
  // log.logVectors(sim.landmarks_);

  while (sim.run())
  {
    log.log(sim.t_);
    log.logVectors(sim.state_.arr);
  }
  printf("Initial values:\n");
  //printState(est.position_meas, est.position_optimized, est.landmarks_optimized,
             //sim.landmarks_);
  printState(est.position_optimized, est.landmarks_optimized,
             sim.landmarks_);

  est.solve();
  printf("Final values:\n");
  //printState(est.position_meas, est.position_optimized, est.landmarks_optimized,
             //sim.landmarks_);
  printState(est.position_optimized, est.landmarks_optimized,
             sim.landmarks_);

  return 0;
}
