#pragma once

#include <vector>

class RangeBearingMeas
{
public:
  double t; // time stamp of these measurements
  double range_variance;
  double bearing_variance;
  double min_range;
  double max_range;

  std::vector<double> ranges; // landmark distances
  std::vector<double> bearings; // landmark bearings from the robot (-pi, pi]

  void reserve(const int& N)
  {
    ranges.reserve(N);
    bearings.reserve(N);
  }

  void clear()
  {
    ranges.clear();
    bearings.clear();
  }
};

