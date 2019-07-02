#include <iostream>
#include <string>

#include "unicycle_landmarks/full_slam_estimator.h"
#include "unicycle_landmarks/unicycle_sim.h"

#include "utils/logger.h"
#include "utils/frame_helper.h"

int main()
{
  std::string sim_params_yaml_file = "../params/unicycle_sim/sim_params.yaml";
  bool show_progress_bar = true;
  UnicycleSimulator sim(sim_params_yaml_file, show_progress_bar);
  Logger log("/tmp/unicycle_full_slam.bin");

  // Estimator
  FullSLAMEstimator est;
  sim.registerEstimator(&est);

  // Log landmark info
  log.log(static_cast<double>(sim.num_landmarks_));
  log.logVectors(sim.landmarks_);

  while (sim.run())
  {
    //std::cout << "step" << std::endl;
    log.log(sim.t_);
    log.logVectors(sim.state_.arr);
  }
  est.solve();

  return 0;
}
