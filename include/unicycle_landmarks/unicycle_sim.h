#pragma once

#include <random>
#include <vector>

#include "unicycle_landmarks/estimator_base.h"
#include "unicycle_landmarks/state.h"

#include "utils/progress_bar.h"
#include "utils/yaml_helpers.h"

class UnicycleSimulator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  UnicycleSimulator(std::string filename, bool prog_indicator = false,
                    uint64_t seed = 0);
  ~UnicycleSimulator();

  void registerEstimator(EstimatorBase* estimator);

  bool run();
  void dynamics();
  void sensors();

  double wrapAngle(double theta);

  void loadParams();
  void setupLandmarks();

  // Sensors
  void initOdomSensor();
  void updateOdomSensor();
  void initRangeBearingSensor();
  void updateRangeBearingSensor();

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

  // Estimator
  EstimatorBase* est_ = nullptr;

  // Landmarks
  int num_landmarks_;
  Eigen::MatrixXd landmarks_;

  // Odometry Sensor
  bool odom_sensor_;
  double odom_vel_std_;
  double odom_omega_std_;
  Eigen::Vector2d odom_meas_;
  Eigen::Matrix2d odom_R_;

  // Range Bearing Sensor
  bool range_bearing_sensor_;
  double rbs_range_std_;
  double rbs_min_range_;
  double rbs_max_range_;
  double rbs_bearing_std_;
  RangeBearingMeas rbs_meas_;
};
