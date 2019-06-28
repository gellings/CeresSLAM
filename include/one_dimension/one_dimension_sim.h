#pragma once

#include <random>
#include <vector>

#include "one_dimension/estimator_base.h"
#include "one_dimension/state.h"
#include "one_dimension/range_meas.h"

#include "utils/progress_bar.h"
#include "utils/yaml_helpers.h"

class OneDimensionSimulator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OneDimensionSimulator(std::string filename, bool prog_indicator = false,
                    uint64_t seed = 0);
  ~OneDimensionSimulator();

  void registerEstimator(EstimatorBase* estimator);

  bool run();
  void dynamics();
  void sensors();

  double wrapAngle(double theta);

  void loadParams();
  void setupLandmarks();

  // Sensors
  void initPositionSensor();
  void updatePositionSensor();
  void initOdomSensor();
  void updateOdomSensor();
  void initRangeSensor();
  void updateRangeSensor();

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
  std::normal_distribution<double> normal_;

  // Estimator
  EstimatorBase* est_;

  // Landmarks
  int num_landmarks_;
  std::vector<double> landmarks_;

  // Position Sensor
  bool position_sensor_;
  double position_stddev_;
  double position_R_;
  double position_meas_;

  // Odometry Sensor
  bool odom_sensor_;
  double odom_vel_std_;
  double odom_R_;
  double odom_meas_;

  // Range Sensor
  bool range_sensor_;
  double range_std_;
  double min_range_;
  double max_range_;
  RangeMeas range_meas_;
};
