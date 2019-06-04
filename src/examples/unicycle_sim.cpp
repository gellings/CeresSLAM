#include <iostream>
#include <string>

#include "utils/progress_bar.h"
#include "utils/logger.h"
#include "utils/yaml_helpers.h"
#include "utils/frame_helper.h"

int main()
{
  std::cout << "Hellllo" << std::endl;

  // Test Progress Bar
  double tmax = 20.;
  double dt = 0.000001;
  double t = 0.;

  ProgressBar prog_;

  prog_.init(std::round(tmax/dt), 40);

  while (t < tmax)
  {
    prog_.print(t/dt);
    t += dt;
  }
  prog_.finished();
  std::cout << std::endl;

  // Test Logger
  Logger log("test_log.bin");
  double a = 5.34;
  log.log(a);

  std::string key = "tmax";
  std::string filename = "../params/unicycle_sim/sim_params.yaml";
  double tmax_2 = 0.;
  bool success = get_yaml_node(key, filename, tmax_2);

  std::cout << "tmax: " << tmax_2 << std::endl;

  double roll = 0.;
  double pitch = 0.;
  double yaw = 0.45;

  Eigen::Matrix3d Rotm = frame_helper::R_v_to_b(roll, pitch, yaw);
  std::cout << "Rotm: " << Rotm << std::endl;

  return 0;
}
