#ifndef __KF_H
#define __KF_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/rng/rng.h>

using namespace MatrixWrapper;

class Kf
{
private:
  static const uint32_t STATES_ = 9;
  ColumnVector y_k_k_1, y_k_k, x_k_k, x_k_k_1, y;
  Matrix Y_k_k_1, Y_k_k, H_gps, H_enc, Qc, G, Q, Y, F;
  bool predict_;

public:
  Kf();
  void Predict(ColumnVector fn, Matrix Cnb, double dt);
  void UpdateGPS(ColumnVector ins, ColumnVector meas, Matrix R);
  void UpdateEncoder(ColumnVector meas, Matrix R);
  void UpdateVicon(ColumnVector meas, Matrix R);
  ColumnVector GetState();
};

#endif
