#ifndef __INS_H
#define __INS_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/rng/rng.h>
#include <cmath>

using namespace MatrixWrapper;

class Ins
{
private:
  ColumnVector state_;
  Matrix Cnb;
  static const int NUM_STATES_ = 9;
  bool imu_rxed_;
  sensor_msgs::Imu imu_msg_last_;
public:

};

#endif
