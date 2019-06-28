#include <iostream>
#include <string>

#include "one_dimension/online_slam_estimator.h"
#include "one_dimension/one_dimension_sim.h"

#include "utils/logger.h"
#include "utils/frame_helper.h"

using std::vector;

int main()
{
  std::string sim_params_yaml_file = "../params/one_dimension/sim_params.yaml";
  bool show_progress_bar = true;
  OneDimensionSimulator sim(sim_params_yaml_file, show_progress_bar);
  Logger log("/tmp/one_dim_sim.bin");

  // Estimator
  OnlineSLAMEstimator est;
  sim.registerEstimator(&est);

  // Log landmark info
  // log.log(static_cast<double>(sim.num_landmarks_));
  // log.logVectors(sim.landmarks_);

  while (sim.run())
  {
    log.log(sim.t_);
    log.logVectors(sim.state_.arr);
  }

  return 0;
}
