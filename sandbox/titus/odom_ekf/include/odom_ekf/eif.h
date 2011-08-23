#ifndef __EIF_H
#define __EIF_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/rng/rng.h>

using namespace MatrixWrapper;

class Eif
{
private:
  static const uint32_t STATES_ = 9;//4;
  ColumnVector y_k_k_1, y_k_k, x_k_k, x_k_k_1, F;
  Matrix Y_k_k_1, Y_k_k, dF, H_gps, H_enc, Cnb;
  sensor_msgs::Imu last_msg_;
  bool first_msg_;
  
  void CalcDCM();
  void CalcDCM(double yaw, double pitch, double roll);
public:
  Eif();
  void Predict(sensor_msgs::Imu msg);
  //void Predict(double ax, double wz, Matrix Q);
  void UpdateGPS(ColumnVector meas, Matrix R);
  void UpdateEncoder(ColumnVector meas, Matrix R);
  void UpdateVicon(ColumnVector meas, Matrix R);
  ColumnVector GetEstimate();
};

#endif
