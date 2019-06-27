#include <iostream>
#include <string>

#include "one_dimension/full_slam_estimator.h"
#include "one_dimension/one_dimension_sim.h"

#include "utils/logger.h"
#include "utils/frame_helper.h"

using std::vector;

void printState(const vector<double>& odom_readings,
                const vector<double>& range_readings)
{
  double robot_location = 0.;
  printf("pose: location     odom    range  r.error  o.error\n");
  for (int i = 0; i < odom_readings.size(); i++)
  {
    robot_location += odom_readings[i];
    const double range_err = 0.;
    const double odom_err = 0.;
    printf("%4d: %8.3f %8.3f %8.3f %8.3f %8.3f\n",
           static_cast<int>(i), robot_location, odom_readings[i],
           range_readings[i], range_err, odom_err);
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
  log.log(static_cast<double>(sim.num_landmarks_));
  log.logVectors(sim.landmarks_);

  while (sim.run())
  {
    // std::cout << "step" << std::endl;
    log.log(sim.t_);
    log.logVectors(sim.state_.arr);
  }
  printf("Initial values:\n");
  printState(est.odom_values, est.range_readings);

  est.solve();
  printf("Final values:\n");
  printState(est.odom_values, est.range_solved);

  return 0;
}
