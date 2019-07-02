#include <iostream>
#include <string>

#include "one_dimension/online_slam_estimator.h"
#include "one_dimension/one_dimension_sim.h"

#include "utils/logger.h"
#include "utils/frame_helper.h"

// TODO remove
#include "utils/circular_buffer.h"

using std::vector;

//void printBuf(const CircularBuffer<double>& buff)
//{
  //std::cout << "Buffer:" << std::endl;
  //for (int i = 0; i < buff.size(); i++)
  //{
    //std::cout << "i: " << i << " val: " << buff[i] << std::endl;
  //}
//}
int main()
{
  //CircularBuffer<double> circ_buff(5);

  //circ_buff.put(0.);
  //for (int i = 0; i < 50; i++)
  //{
    //double last = circ_buff.get_newest();
    //std::cout << "last: " << last << std::endl;
    //circ_buff.put(static_cast<double>(i));
    //circ_buff.put(last + 1.);
    //printBuf(circ_buff);
  //}

  std::string sim_params_yaml_file = "../params/one_dimension/sim_params.yaml";
  bool show_progress_bar = true;
  OneDimensionSimulator sim(sim_params_yaml_file, show_progress_bar);
  Logger log("/tmp/online_1d_slam.bin");

  // Estimator
  OnlineSLAMEstimator est;
  sim.registerEstimator(&est);

  // Log landmark info
  log.log(static_cast<double>(sim.num_landmarks_));

  while (sim.run())
  {
    log.log(sim.t_);

    // Log true x, v
    log.logVectors(sim.state_.arr);

    // Log estimated x, v
    log.log(est.getEstimatedX());
    log.log(est.getEstimatedV());

    // Log true landmarks
    for (auto true_lm : sim.landmarks_)
    {
      log.log(true_lm);
    }

    // Log estimated landmarks
    for (auto est_lm : est.getEstimatedLandmarks())
    {
      log.log(est_lm);
    }
  }

  return 0;
}
