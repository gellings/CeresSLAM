#include "unicycle_landmarks/unicycle_sim.h"

#include "utils/progress_bar.h"
#include "utils/yaml_helpers.h"

UnicycleSimulator::UnicycleSimulator(std::string filename, bool prog_indicator,
                                     uint64_t seed) :
  seed_(seed == 0 ? std::chrono::system_clock::now().time_since_epoch().count() : seed),
  rng_(seed_),
  uniform_(0.0, 1.0),
  param_filename_(filename),
  prog_indicator_(prog_indicator)
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
  // ODE
  // xdot = v * cos(theta)
  // ydot = v * sin(theta)
  // thetadot = omega
  const double pos_x = state_.x();
  const double pos_y = state_.y();
  const double theta = state_.theta();
  const double v = state_.v();
  const double omega = state_.omega();

  state_.x() += v * cos(theta) * dt_;
  state_.y() += v * sin(theta) * dt_;
  state_.theta() += omega * dt_;

  wrapAngle();
}

void UnicycleSimulator::wrapAngle()
{
  while(state_.theta() > M_PI)
  {
    state_.theta() -= 2 * M_PI;
  }
  while(state_.theta() < -M_PI)
  {
    state_.theta() += 2 * M_PI;
  }
}

void UnicycleSimulator::loadParams()
{
  get_yaml_node("tmax", param_filename_, tmax_);
  get_yaml_node("dt", param_filename_, dt_);
  get_yaml_node("seed", param_filename_, seed_);
  if (seed_ == 0)
    seed_ = std::chrono::system_clock::now().time_since_epoch().count();
  rng_ = std::default_random_engine(seed_);
  srand(seed_);

  if (prog_indicator_)
    prog_.init(std::round(tmax_/dt_), 40);

  setupLandmarks();

  // Setup motion model
  get_yaml_eigen("x0", param_filename_, state_.arr);
}

void UnicycleSimulator::setupLandmarks()
{
  // Setup landmarks
  get_yaml_node("num_landmarks", param_filename_, num_landmarks_);

  landmarks_.resize(num_landmarks_, 2);

  double xmin, xmax, ymin, ymax;
  get_yaml_node("xmin", param_filename_, xmin);
  get_yaml_node("xmax", param_filename_, xmax);
  get_yaml_node("ymin", param_filename_, ymin);
  get_yaml_node("ymax", param_filename_, ymax);

  // Get random positions for landmarks
  for (unsigned int i = 0; i < num_landmarks_; i++)
  {
    // random x
    landmarks_(i, 0) = (xmax - xmin) * uniform_(rng_) + xmin;

    // random y
    landmarks_(i, 1) = (ymax - ymin) * uniform_(rng_) + ymin;
  }
}
