#include "unicycle_landmarks/unicycle_sim.h"

#include "utils/progress_bar.h"
#include "utils/yaml_helpers.h"

UnicycleSimulator::UnicycleSimulator(std::string filename, bool prog_indicator,
                                     uint64_t seed)
  : param_filename_(filename), prog_indicator_(prog_indicator)
{
  t_ = 0.;
  // TODO
  // seed ...

  loadParams();
}

UnicycleSimulator::~UnicycleSimulator()
{
  if (prog_indicator_)
    std::cout << std::endl;
}

bool UnicycleSimulator::run()
{
  if (t_ < tmax_ - dt_ / 2.0)  // Subtract half time step to prevent occasional
                               // extra iteration
  {
    t_ += dt_;
    dynamics();
    if (prog_indicator_)
      prog_.print(t_ / dt_);
    return true;
  }
  else
  {
    if (prog_indicator_)
      prog_.finished();
    return false;
  }
}

void UnicycleSimulator::dynamics()
{
  state_.x() += state_.v() * dt_;
}

void UnicycleSimulator::loadParams()
{
  get_yaml_node("tmax", param_filename_, tmax_);
  get_yaml_node("dt", param_filename_, dt_);

  if (prog_indicator_)
    prog_.init(std::round(tmax_/dt_), 40);

  // Start from 0 for now
  state_.arr.setZero();
  double vel;
  get_yaml_node("vel", param_filename_, vel);
  state_.v() = vel;
}
