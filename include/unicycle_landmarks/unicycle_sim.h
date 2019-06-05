#pragma once

#include <random>
#include <vector>

#include "unicycle_landmarks/state.h"

#include "utils/progress_bar.h"
#include "utils/yaml_helpers.h"

class UnicycleSimulator
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  UnicycleSimulator(std::string filename, bool prog_indicator=false, uint64_t seed=0);
  ~UnicycleSimulator();

  bool run();
  void dynamics();
  void wrapAngle();

  void loadParams();
  void setupLandmarks();

  ProgressBar prog_;
  bool prog_indicator_;
  std::string param_filename_;

  double tmax_;
  double dt_;

  double t_;
  State state_;

  // Random number Generation
  uint64_t seed_;
  std::default_random_engine rng_;
  std::uniform_real_distribution<double> uniform_;

  // Landmarks
  int num_landmarks_;
  Eigen::MatrixXd landmarks_;
};
