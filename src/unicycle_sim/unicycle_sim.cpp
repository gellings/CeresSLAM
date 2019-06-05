#include "unicycle_landmarks/unicycle_sim.h"

#include "utils/progress_bar.h"
#include "utils/yaml_helpers.h"

UnicycleSimulator::UnicycleSimulator(std::string filename, bool prog_indicator,
                                     uint64_t seed)
  : seed_(seed == 0 ?
              std::chrono::system_clock::now().time_since_epoch().count() :
              seed)
  , rng_(seed_)
  , uniform_(0.0, 1.0)
  , param_filename_(filename)
  , prog_indicator_(prog_indicator)
{
  t_ = 0.;

  loadParams();
}

UnicycleSimulator::~UnicycleSimulator()
{
  if (prog_indicator_)
    std::cout << std::endl;
}

void UnicycleSimulator::registerEstimator(EstimatorBase* estimator)
{
  est_ = estimator;
}

bool UnicycleSimulator::run()
{
  if (t_ < tmax_ - dt_ / 2.0)  // Subtract half time step to prevent occasional
                               // extra iteration
  {
    t_ += dt_;
    dynamics();

    if (est_)
    {
      sensors();
    }

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

  // Note theta is defined as 0 pointing along the x axis, and positive ccw,
  // toward the y axis
  const double pos_x = state_.x();
  const double pos_y = state_.y();
  const double theta = state_.theta();
  const double v = state_.v();
  const double omega = state_.omega();

  state_.x() += v * cos(theta) * dt_;
  state_.y() += v * sin(theta) * dt_;
  state_.theta() += omega * dt_;

  state_.theta() = wrapAngle(state_.theta());
}

void UnicycleSimulator::sensors()
{
  if (odom_sensor_)
  {
    updateOdomSensor();
  }

  if (range_bearing_sensor_)
  {
    updateRangeBearingSensor();
  }
}

double UnicycleSimulator::wrapAngle(double theta)
{
  while (theta > M_PI)
  {
    theta -= 2 * M_PI;
  }
  while (theta <= -M_PI)
  {
    theta += 2 * M_PI;
  }

  return theta;
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
    prog_.init(std::round(tmax_ / dt_), 40);

  setupLandmarks();

  // Setup motion model
  get_yaml_eigen("x0", param_filename_, state_.arr);

  // Setup sensors
  get_yaml_node("odometry_sensor", param_filename_, odom_sensor_);
  if (odom_sensor_)
  {
    initOdomSensor();
  }

  get_yaml_node("range_bearing_sensor", param_filename_, range_bearing_sensor_);
  if (range_bearing_sensor_)
  {
    initRangeBearingSensor();
  }
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

void UnicycleSimulator::initOdomSensor()
{
  get_yaml_node("velocity_std", param_filename_, odom_vel_std_);
  get_yaml_node("omega_std", param_filename_, odom_omega_std_);

  odom_R_.setZero();
  odom_R_(0, 0) = odom_vel_std_ * odom_vel_std_;
  odom_R_(1, 1) = odom_omega_std_ * odom_omega_std_;
}

void UnicycleSimulator::updateOdomSensor()
{
  odom_meas_(0) = state_.v();
  odom_meas_(1) = state_.omega();

  est_->odometryCallback(t_, odom_meas_, odom_R_);
}

void UnicycleSimulator::initRangeBearingSensor()
{
  get_yaml_node("range_std", param_filename_, rbs_range_std_);
  get_yaml_node("min_range", param_filename_, rbs_min_range_);
  get_yaml_node("max_range", param_filename_, rbs_max_range_);
  get_yaml_node("bearing_std", param_filename_, rbs_bearing_std_);

  rbs_meas_.reserve(num_landmarks_);
  rbs_meas_.range_variance = rbs_range_std_ * rbs_range_std_;
  rbs_meas_.bearing_variance = rbs_bearing_std_ * rbs_bearing_std_;
  rbs_meas_.min_range = rbs_min_range_;
  rbs_meas_.max_range = rbs_max_range_;
}

void UnicycleSimulator::updateRangeBearingSensor()
{
  rbs_meas_.clear();

  for (unsigned int i = 0; i < num_landmarks_; i++)
  {
    // calc range
    const double x_diff = landmarks_(i, 0) - state_.x();
    const double y_diff = landmarks_(i, 1) - state_.y();
    const double range = sqrt(x_diff * x_diff + y_diff * y_diff);
    rbs_meas_.ranges.push_back(range);

    // calc bearing
    const double ang_to_lm = atan2(y_diff, x_diff);
    const double bearing = ang_to_lm - state_.theta();
    rbs_meas_.bearings.push_back(wrapAngle(bearing));
  }

  est_->rangeBearingCallback(t_, rbs_meas_);
}
