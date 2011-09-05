#ifndef __INS_H
#define __INS_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/rng/rng.h>
#include "LinearMath/btMatrix3x3.h"
#include "tf/tf.h"
#include <math.h>
#include <cmath>

using namespace MatrixWrapper;

class Ins
{
private:
  ColumnVector state_, state_last_, state_last2_, fn_, fbias_, wbias_, g_;
  Matrix Cnb_;
  static const int NUM_STATES_ = 9;
  bool initialized_;
  sensor_msgs::Imu imu_msg_last_;
  double dt_;
  
  RowVector CrossProduct(RowVector a, RowVector b);
  RowVector Normalize(RowVector a);
  void CorrectDCM();
  void CalcDCM(double yaw, double pitch, double roll);
public:
  Ins();
  bool Initialize(sensor_msgs::Imu msg);
  void Integrate(sensor_msgs::Imu msg);
  ColumnVector GetState();
  ColumnVector GetLastFn();
  Matrix GetCnb();
  double GetDt();
  void Correct(ColumnVector errors);
};

#endif
