#include "one_dimension/one_dimension_sim.h"

#include "utils/progress_bar.h"
#include "utils/yaml_helpers.h"

OneDimensionSimulator::OneDimensionSimulator(std::string filename,
                                             bool prog_indicator, uint64_t seed)
  : seed_(seed == 0 ?
              std::chrono::system_clock::now().time_since_epoch().count() :
              seed)
  , rng_(seed_)
  , uniform_(0.0, 1.0)
  , normal_(0., 1.)
  , param_filename_(filename)
  , prog_indicator_(prog_indicator)
{
  est_ = nullptr;
  t_ = 0.;

  loadParams();
}

OneDimensionSimulator::~OneDimensionSimulator()
{
  if (prog_indicator_)
    std::cout << std::endl;
}

void OneDimensionSimulator::registerEstimator(EstimatorBase* estimator)
{
  est_ = estimator;
}

bool OneDimensionSimulator::run()
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

void OneDimensionSimulator::dynamics()
{
  // ODE
  // xdot = v
  // vdot = 0

  const double v = state_.v();

  state_.x() += v * dt_;
}

void OneDimensionSimulator::sensors()
{
  if (odom_sensor_)
  {
    updateOdomSensor();
  }

  if (range_sensor_)
  {
    updateRangeSensor();
  }
}

void OneDimensionSimulator::loadParams()
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

  get_yaml_node("range_sensor", param_filename_, range_sensor_);
  if (range_sensor_)
  {
    initRangeSensor();
  }
}

void OneDimensionSimulator::setupLandmarks()
{
  // Setup landmarks
  get_yaml_node("num_landmarks", param_filename_, num_landmarks_);

  landmarks_.resize(num_landmarks_);

  double xmin, xmax, ymin, ymax;
  get_yaml_node("xmin", param_filename_, xmin);
  get_yaml_node("xmax", param_filename_, xmax);

  // Get random positions for landmarks
  for (unsigned int i = 0; i < num_landmarks_; i++)
  {
    // random x
    landmarks_(i) = (xmax - xmin) * uniform_(rng_) + xmin;
  }
}

void OneDimensionSimulator::initOdomSensor()
{
  get_yaml_node("velocity_std", param_filename_, odom_vel_std_);

  odom_R_ = odom_vel_std_ * odom_vel_std_;
}

void OneDimensionSimulator::updateOdomSensor()
{
  odom_meas_ = state_.v() + odom_vel_std_ * normal_(rng_);

  est_->odometryCallback(t_, odom_meas_, odom_R_);
}

void OneDimensionSimulator::initRangeSensor()
{
  get_yaml_node("range_std", param_filename_, range_std_);
  get_yaml_node("min_range", param_filename_, min_range_);
  get_yaml_node("max_range", param_filename_, max_range_);

  range_meas_.reserve(num_landmarks_);
  range_meas_.range_variance = range_std_ * range_std_;
  range_meas_.min_range = min_range_;
  range_meas_.max_range = max_range_;
}

void OneDimensionSimulator::updateRangeSensor()
{
  range_meas_.clear();

  for (unsigned int i = 0; i < num_landmarks_; i++)
  {
    // calc range
    const double x_diff = landmarks_(i) - state_.x();
    const double range = sqrt(x_diff * x_diff);
    const double range_meas = range + range_std_ * normal_(rng_);
    range_meas_.ranges.push_back(range_meas);
  }

  est_->rangeCallback(t_, range_meas_);
}
