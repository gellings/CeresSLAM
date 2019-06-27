#pragma once

#include <vector>

class RangeMeas
{
public:
  double t; // time stamp of these measurements
  double range_variance;
  double min_range;
  double max_range;

  std::vector<double> ranges; // landmark distances

  void reserve(const int& N)
  {
    ranges.reserve(N);
  }

  void clear()
  {
    ranges.clear();
  }
};

