#pragma once

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

  void loadParams();

  ProgressBar prog_;
  bool prog_indicator_;
  std::string param_filename_;

  double tmax_;
  double dt_;

  double t_;
  State state_;



};
